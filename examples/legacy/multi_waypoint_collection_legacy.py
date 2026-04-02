import os
import time
import numpy as np
import cv2
import socket
from datetime import datetime
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import pyrealsense2 as rs
import pyzed.sl as sl
import traceback # For detailed error printing

# === Configuration ===
# Network
ROBOT_IP = "192.168.192.7"
ROBOT_PORT = 30002 # Port for sending URScript commands

# Data Collection
# !! Adjust these !!
FPS = 5            # Target collection frequency (Increased to 10)
RECORDING_DURATION = 17 # Desired duration of the *saved* data (seconds)
CAMERA_WARMUP_SECONDS = 1 # Skip data from the first N seconds due to camera warmup
# --------------------

# Calculated parameters
SLEEP_INTERVAL = 1.0 / FPS
INITIAL_FRAMES_TO_SKIP = int(CAMERA_WARMUP_SECONDS * FPS)
SAVED_FRAMES_NUM = int(RECORDING_DURATION * FPS) # How many frames we want to save
TOTAL_ITERATIONS = SAVED_FRAMES_NUM + INITIAL_FRAMES_TO_SKIP # Total loop runs

SAVE_ROOT = "E:/UR5_test/datasets" # Updated save root name

# Robot Task Parameters
acceleration = 0.5  # m/s^2
velocity = 0.15   # m/s

# === 任务描述 ===
# 这组点位适用于：A3_to_B1__pick up block and place it into the box

# === 机器人任务简化位姿与路径 ===
# 定义关键位姿（字符串形式）

initial_pose_str = "p[-0.5602, -0.2251, 0.4755, 3.090, 0.5860, -0.2650]" # 起始/Home 位姿 (使用默认值) 
grab_pose_str          = "p[-0.6876, -0.1642, 0.1615, -2.9149, -0.9682, -0.1437]" # 实际抓取点
place_pose_str          = "p[-0.6589, -0.5072, 0.2091, -2.8821, -1.0923, 0.0182]" # 实际放置点

# 定义中间位姿列表，用于连接关键点之间的路径
# --- 从初始位姿到抓取位姿的路径 ---
# 在这里添加任意数量的 p[...] 字符串。可留空列表 [] 以进行直接移动。
intermediate_poses_initial_to_grab_strs = [
    "p[-0.5871, -0.1553, 0.5055, 3.0597, 0.3873, -0.0532]",
    "p[-0.6671, -0.1609, 0.4356, 3.0553, 0.3835, 0.0668]",
    "p[-0.7149, -0.1243, 0.3438, 3.0426, 0.2907, 0.2643]",
    "p[-0.7111, -0.1563, 0.2556, 2.8827, 0.8818, 0.3119]",
]

# --- 从抓取位姿到放置位姿的路径 ---
# 在这里添加任意数量的 p[...] 字符串。可留空列表 [] 以进行直接移动。
intermediate_poses_grab_to_place_strs = [
    "p[-0.6752, -0.1630, 0.2391, -2.8959, -0.9539, 0.0232]",
    "p[-0.6317, -0.1592, 0.3730, -2.8410, -0.9218, 0.3159]",
    "p[-0.5912, -0.2883, 0.4127, -2.9307, -0.8712, 0.1216]",
    "p[-0.5930, -0.4369, 0.3544, -2.8352, -1.1616, 0.0871]",
    "p[-0.6467, -0.4845, 0.2953, -2.8241, -1.1909, 0.0953]",
]


# === URScript Generation Function ===
def generate_urscript_simplified_path(
    initial_p_str,
    grab_p_str,
    place_p_str,
    inter_initial_to_grab_strs,
    inter_grab_to_place_strs,
    accel,
    vel
):
    """Generates the URScript string based on simplified flexible pose inputs."""
    script_parts = []
    script_parts.append("def my_robot_program():")

    # Define all poses as variables within the URScript
    script_parts.append(f"  var_initial_p = {initial_p_str}")
    script_parts.append(f"  var_grab_p = {grab_p_str}")
    script_parts.append(f"  var_place_p = {place_p_str}")

    # Define intermediate poses as variables
    intermediate_vars_initial_to_grab = []
    for i, pose_str in enumerate(inter_initial_to_grab_strs):
        var_name = f"var_inter_init_grab_{i}"
        script_parts.append(f"  {var_name} = {pose_str}")
        intermediate_vars_initial_to_grab.append(var_name)

    intermediate_vars_grab_to_place = []
    for i, pose_str in enumerate(inter_grab_to_place_strs):
        var_name = f"var_inter_grab_place_{i}"
        script_parts.append(f"  {var_name} = {pose_str}")
        intermediate_vars_grab_to_place.append(var_name)

    # ----- Initialize Gripper to Full Open (Preset 2) -----
    script_parts.append("\n  # ----- Initialize Gripper to Full Open -----")
    script_parts.append("  set_tool_digital_out(0, True)")   # Tool DOUT0 High
    script_parts.append("  set_tool_digital_out(1, False)")  # Tool DOUT1 Low
    script_parts.append("  set_digital_out(0, True)")        # Base DOUT0 High
    script_parts.append("  set_digital_out(1, False)")       # Base DOUT1 Low
    script_parts.append("  sleep(0.2)") # Wait for gripper

    # ----- Move Sequence -----
    script_parts.append("\n  # ----- Move Sequence -----")

    # Move to initial pose
    script_parts.append(f"  movel(var_initial_p, a={accel}, v={vel})")
    script_parts.append("  sleep(0.2)") # Small pause after initial move

    # Move through intermediate points from initial to grab
    script_parts.append("  # Path from initial to grab")
    for var_name in intermediate_vars_initial_to_grab:
        script_parts.append(f"  movel({var_name}, a={accel}, v={vel})")
        # script_parts.append("  sleep(0.01)") # Small delay between intermediates

    # Move to actual grab point
    script_parts.append(f"  movel(var_grab_p, a={accel}, v={vel})")
    script_parts.append("  sleep(0.2)") # Pause at grab point

    # ----- Gripper Control: Close (Preset 1) -----
    script_parts.append("\n  # ----- Gripper Control: Close -----")
    script_parts.append("  set_tool_digital_out(0, False)")  # Tool DOUT0 Low
    script_parts.append("  set_tool_digital_out(1, False)")  # Tool DOUT1 Low
    script_parts.append("  set_digital_out(0, False)")       # Base DOUT0 Low
    script_parts.append("  set_digital_out(1, False)")       # Base DOUT1 Low
    script_parts.append("  sleep(0.5)") # Wait for gripper

    # Move through intermediate points from grab to place
    script_parts.append("\n  # Path from grab to place")
    for var_name in intermediate_vars_grab_to_place:
        script_parts.append(f"  movel({var_name}, a={accel}, v={vel})")
        # script_parts.append("  sleep(0.01)") # Small delay between intermediates

    # Move to actual place point
    script_parts.append(f"  movel(var_place_p, a={accel}, v={vel})")
    script_parts.append("  sleep(0.2)") # Pause at place point

    # ----- Gripper Control: Open (Preset 2) -----
    script_parts.append("\n  # ----- Gripper Control: Open -----")
    script_parts.append("  set_tool_digital_out(0, True)")   # Tool DOUT0 High
    script_parts.append("  set_tool_digital_out(1, False)")  # Tool DOUT1 Low
    script_parts.append("  set_digital_out(0, True)")        # Base DOUT0 High
    script_parts.append("  set_digital_out(1, False)")       # Base DOput1 Low
    script_parts.append("  sleep(0.5)") # Wait for gripper

    # --- REMOVED: Optional: Move back to initial pose after placing ---
    # script_parts.append("\n  # Move back to initial pose")
    # script_parts.append(f"  movel(var_initial_p, a={accel}, v={vel})")
    # script_parts.append("  sleep(0.2)")


    # ----- Optional: Add a signal for completion -----
    # Example: script_parts.append("  set_standard_digital_out(7, True)")
    # The Python script could then monitor this output via RTDE to stop recording precisely.

    script_parts.append("end")
    script_parts.append("my_robot_program()")

    return "\n".join(script_parts)


# === Generate the URScript for this specific run ===
urscript_command = generate_urscript_simplified_path(
    initial_pose_str,
    grab_pose_str,
    place_pose_str,
    intermediate_poses_initial_to_grab_strs,
    intermediate_poses_grab_to_place_strs,
    acceleration,
    velocity
)

# # Optional: Print the generated URScript for debugging
# print("--- Generated URScript ---")
# print(urscript_command)
# print("--------------------------")


# === Initialize Resources ===
pipeline = None
zed = None
rtde_r = None
zed_init_success = False
rs_init_success = False
rtde_init_success = False

try:
    # Create Save Paths
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = os.path.join(SAVE_ROOT, f"data_{timestamp}")
    image_dir_zed = os.path.join(save_dir, "image0") # ZED images
    image_dir_d435 = os.path.join(save_dir, "image1") # D435 images
    os.makedirs(image_dir_zed, exist_ok=True)
    os.makedirs(image_dir_d435, exist_ok=True)
    print(f"📁 Save directory: {save_dir}")

    # Initialize ZED Camera
    print("🔄 Initializing ZED Camera...")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 15 # Camera internal FPS (Keep higher or match collection FPS)
    # Added check for opening success
    zed_open_status = zed.open(init_params)
    if zed_open_status == sl.ERROR_CODE.SUCCESS:
        left_image = sl.Mat()
        runtime_params = sl.RuntimeParameters()
        zed_init_success = True
        print("✅ ZED Camera initialized.")
    else:
        print(f"❌ ZED Camera failed to open: {zed_open_status}")
        # Consider exiting if ZED is essential, or continue if D435 is sufficient

    # Initialize RealSense D435
    print("🔄 Initializing RealSense D435 Camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    # Ensure RealSense FPS is >= collection FPS if possible
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    try:
        pipeline.start(config)
        rs_init_success = True
        print("✅ RealSense D435 Camera initialized.")
    except Exception as e:
        print(f"❌ RealSense D435 Camera failed to start: {e}")
        traceback.print_exc()


    # Initialize RTDE Receive Interface
    print("🔄 Initializing RTDE Receiver...")
    rtde_r = RTDEReceive(ROBOT_IP)
    # Check initial connection (optional but good practice)
    if rtde_r.isConnected():
        rtde_init_success = True
        print("✅ RTDE Receiver initialized and connected.")
    else:
         print("❌ RTDE Receiver initialized but failed to connect initially.")
         # Decide if script should exit if RTDE isn't connected from the start
         # exit(1)


    # === Main Execution ===
    robot_data = [] # List to store the *saved* robot data

    # Check if essential devices are ready before starting
    if not rtde_init_success:
         print("❌ Cannot proceed: RTDE Receiver failed to connect.")
         raise ConnectionError("RTDE connection failed during initialization.") # Raise error to trigger finally block

    # Check if *at least one* camera is initialized
    if not zed_init_success and not rs_init_success:
         print("❌ Cannot proceed: Neither ZED nor RealSense camera initialized successfully.")
         raise IOError("No camera could be initialized.") # Raise error


    # 1. Send generated URScript command to start the robot task
    print(f"🤖 Attempting to connect to robot {ROBOT_IP}:{ROBOT_PORT} to send task...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5.0) # Add a timeout for connection
            s.connect((ROBOT_IP, ROBOT_PORT))
            s.sendall((urscript_command + '\n').encode('ascii'))
            print("✅ URScript command sent successfully. Robot task should be starting.")
            # Socket is automatically closed by 'with' statement
    except ConnectionRefusedError:
        print(f"❌ Socket connection failed (ConnectionRefusedError): Ensure robot is running and accepts connections on {ROBOT_PORT}.")
        raise # Re-raise the error to be caught by the main except block
    except socket.timeout:
         print(f"❌ Socket connection timed out: Could not connect to robot at {ROBOT_IP}:{ROBOT_PORT} within timeout.")
         raise # Re-raise the error
    except Exception as e:
         print(f"❌ An unexpected error occurred while sending URScript: {e}")
         traceback.print_exc()
         raise # Re-raise any other exceptions

    print(f"\n⏳ Running warmup period and data collection...")
    print(f"   - Target FPS: {FPS}")
    print(f"   - Skipping first {INITIAL_FRAMES_TO_SKIP} frames ({CAMERA_WARMUP_SECONDS} seconds).")
    print(f"   - Recording next {SAVED_FRAMES_NUM} frames ({RECORDING_DURATION} seconds).")
    print(f"   - Total loop iterations: {TOTAL_ITERATIONS}.")

    # Optional small delay after sending command, before intense loop starts
    # time.sleep(0.1) # Can sometimes help ensure robot registers command

    # 2. Start Data Acquisition Loop
    # Loop runs for total iterations including warmup/skipped frames
    for frame_id in range(TOTAL_ITERATIONS):
        start_time = time.time()
        # Determine if this frame should be saved
        should_save_frame = (frame_id >= INITIAL_FRAMES_TO_SKIP)
        # Calculate the index for saving (starts from 0)
        saved_frame_index = frame_id - INITIAL_FRAMES_TO_SKIP

        # --- Always capture data in each iteration to maintain timing ---
        img_zed_bgr = None
        color_image = None
        tcp_pose = None
        tool_do_0_state = None
        tool_do_1_state = None
        gripper_state = 0 # Default state (unknown/transition)
        # frame_data_valid now indicates if robot data was successfully read for this frame
        frame_data_valid = False

        # --- ZED Capture ---
        zed_captured = False
        if zed_init_success and zed.is_opened():
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(left_image, view=sl.VIEW.LEFT)
                img_zed_raw = left_image.get_data()
                # Format conversion (ensure BGR, uint8)
                if img_zed_raw is not None:
                    # Check shape before converting
                    if img_zed_raw.shape[-1] == 4:
                        # ZED often provides BGRA, convert to BGR
                        img_zed_bgr = cv2.cvtColor(img_zed_raw, cv2.COLOR_BGRA2BGR)
                    elif img_zed_raw.shape[-1] == 3:
                        # ZED often provides BGR directly
                        img_zed_bgr = img_zed_raw
                    # Ensure dtype is uint8 if not already
                    if img_zed_bgr is not None and img_zed_bgr.dtype != np.uint8:
                         img_zed_bgr = img_zed_bgr.astype(np.uint8)

                    if img_zed_bgr is not None: zed_captured = True # Mark as captured successfully
            # else: # Optional: Log grab failure only during saving phase
            #     if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: ZED grab failed.")

        # --- D435 Capture ---
        d435_captured = False
        if rs_init_success:
            try:
                # Use shorter timeout based on SLEEP_INTERVAL to avoid blocking too long
                # Add a small buffer, but don't wait excessively if FPS is high
                timeout_ms = max(10, int(SLEEP_INTERVAL * 1000 * 0.8)) # 80% of interval as timeout
                frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
                if frames:
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        color_image = np.asanyarray(color_frame.get_data())
                        d435_captured = True # Mark as captured successfully
                # else: # Optional: Log timeout only during saving phase
                #     if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: D435 wait timed out ({timeout_ms}ms).")
            except Exception as e:
                 if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: Error D435 capture: {e}")
                 traceback.print_exc()


        # --- Robot State Capture (RTDE) ---
        # rtde_captured flag is implicitly handled by frame_data_valid
        if rtde_init_success and rtde_r.isConnected():
             try:
                # *** Using getActualTCPPose and getDigitalOutState directly ***
                # Ensure these variable names are available in your RTDE configuration
                tcp_pose = rtde_r.getActualTCPPose()
                tool_do_0_state = rtde_r.getDigitalOutState(0) # Tool DO 0
                tool_do_1_state = rtde_r.getDigitalOutState(1) # Tool DO 1

                # Check if essential data (TCP Pose) was successfully retrieved
                if tcp_pose is not None:
                    frame_data_valid = True # Mark robot data as valid for this frame
                    # --- Gripper State Inference (Based on Tool Outputs) ---
                    # Ensure DO states were read before inferring
                    if tool_do_0_state is not None and tool_do_1_state is not None:
                        if not tool_do_0_state and not tool_do_1_state: gripper_state = -1 # Closed (Example: MyGripper Preset 1)
                        elif tool_do_0_state and not tool_do_1_state: gripper_state = 1 # Open (Example: MyGripper Preset 2)
                        else: gripper_state = 0 # Other/Unknown state or transition
                    else:
                         gripper_state = 0 # Cannot determine gripper state if DOs not read
                         # Optional: Log if DO states were expected but not received
                         # if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: Failed to get Tool DO states from RTDE.")

                else:
                     # Pose is essential, mark data invalid if None
                     # frame_data_valid is already False from initialization
                     if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: Failed TCP Pose (None).")

             except Exception as e:
                # If any error occurs during RTDE read, mark data invalid
                frame_data_valid = False # Already False
                if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: Error reading RTDE data: {e}")
                traceback.print_exc()
        else:
            # If RTDE is not initialized or disconnected, robot data is invalid
            frame_data_valid = False # Already False
            if should_save_frame: print(f"⚠️ Save Frame {saved_frame_index:04d}: RTDE not connected.")


        # --- Save Data and Aggregate (Only if past the warmup phase) ---
        if should_save_frame:
            print_prefix = f"💾 Save Frame {saved_frame_index:04d}/{SAVED_FRAMES_NUM-1}"

            # Save ZED Image if captured
            if zed_captured and img_zed_bgr is not None:
                zed_img_path = os.path.join(image_dir_zed, f"{saved_frame_index:04d}.jpg")
                try:
                    cv2.imwrite(zed_img_path, img_zed_bgr)
                    zed_save_status = "OK"
                except Exception as e:
                     zed_save_status = f"SaveFAIL({e})"
                     print(f"⚠️ {print_prefix}: Error saving ZED image: {e}")

            else:
                zed_save_status = "FAIL (Capture)"
                # if should_save_frame: print(f"⚠️ {print_prefix}: ZED capture failed.")


            # Save D435 Image if captured
            if d435_captured and color_image is not None:
                color_img_path = os.path.join(image_dir_d435, f"{saved_frame_index:04d}.jpg")
                try:
                    cv2.imwrite(color_img_path, color_image)
                    d435_save_status = "OK"
                except Exception as e:
                    d435_save_status = f"SaveFAIL({e})"
                    print(f"⚠️ {print_prefix}: Error saving D435 image: {e}")

            else:
                 d435_save_status = "FAIL (Capture)"
                 # if should_save_frame: print(f"⚠️ {print_prefix}: D435 capture failed.")


            # Aggregate and store robot data *only* if frame_data_valid is True
            if frame_data_valid:
                # tcp_pose is a list [x, y, z, rx, ry, rz] from getActualTCPPose()
                pose_list = tcp_pose # getActualTCPPose returns a list
                robot_data.append(np.concatenate((pose_list, [gripper_state])))
                print(f"{print_prefix} | Pose Z: {pose_list[2]:.3f} | Grip: {gripper_state} | ZED: {zed_save_status} | D435: {d435_save_status}")
            else:
                 # Log skipped frame during recording phase due to invalid robot data
                 # This covers cases where RTDE was not connected or pose read failed
                 print(f"{print_prefix} | Robot Data: Invalid/Skipped | ZED: {zed_save_status} | D435: {d435_save_status}")


        # --- FPS Control ---
        elapsed = time.time() - start_time
        remaining = SLEEP_INTERVAL - elapsed
        if remaining > 0:
            time.sleep(remaining)
        # else:
            # Only print warning if we are past warmup, as initial frames might take longer
            # if frame_id >= INITIAL_FRAMES_TO_SKIP: # Check should_save_frame
            #      print(f"⚠️ Frame {frame_id:04d}: Loop iteration took longer than SLEEP_INTERVAL ({elapsed:.3f}s)")


    print("\n✅ Data collection loop finished.")

    # 3. Save Collected Robot Data
    if robot_data:
        tcp_save_path = os.path.join(save_dir, "tcp_poses.npy")
        np.save(tcp_save_path, np.array(robot_data))
        print(f"💾 Saved {len(robot_data)} frames of TCP pose & gripper state to: {tcp_save_path}")
        if len(robot_data) < SAVED_FRAMES_NUM:
             print(f"   ⚠️ Warning: Expected {SAVED_FRAMES_NUM} frames to save, but only {len(robot_data)} frames of robot data were valid/saved.")
    else:
        print("⚠️ No valid robot data collected to save.")

except ConnectionRefusedError:
    print(f"❌ Connection failed: Ensure robot is running, allows remote control, and IP {ROBOT_IP}/Port {ROBOT_PORT} are correct.")
    traceback.print_exc()
except socket.timeout:
     print(f"❌ Connection timed out: Could not connect to robot at {ROBOT_IP}:{ROBOT_PORT} within timeout.")
     traceback.print_exc()
except rs.error as e:
    print(f"❌ RealSense Error: {e}")
    traceback.print_exc()
# Specific check for ZED error codes might require checking the type of e
# pyzed.sl.ERROR_CODE objects can sometimes be caught directly, but it's safer
# to catch generic Exception or OSError if direct catch doesn't work
except sl.ERROR_CODE as e: # This might work if pyzed raises this
    print(f"❌ ZED Error Code: {e}")
    traceback.print_exc()
except IOError as e: # Catch the IOError raised if cameras fail init
     print(f"❌ IO Error: {e}")
     traceback.print_exc()
except ConnectionError as e: # Catch the ConnectionError raised if RTDE fails init
     print(f"❌ Connection Error: {e}")
     traceback.print_exc()
except Exception as e:
    print(f"❌ An unexpected error occurred during execution: {e}")
    traceback.print_exc() # Print detailed traceback

finally:
    # 4. Cleanup Resources
    print("\n🧹 Cleaning up resources...")
    if pipeline and rs_init_success: # Check flag before stopping
         try: pipeline.stop(); print("✅ RealSense pipeline stopped.")
         except Exception as e: print(f"⚠️ Error stopping RealSense: {e}")
    # Check if zed object exists and is opened before trying to close
    if zed and zed.is_opened():
         try: zed.close(); print("✅ ZED camera closed.")
         except Exception as e: print(f"⚠️ Error closing ZED: {e}")
    # Check if rtde_r object exists and is connected before trying to disconnect
    if rtde_r and rtde_r.isConnected():
        try: rtde_r.disconnect(); print("✅ RTDE receiver disconnected.")
        except Exception as e: print(f"⚠️ Error disconnecting RTDE: {e}")

    cv2.destroyAllWindows()
    print("🏁 Script finished.")