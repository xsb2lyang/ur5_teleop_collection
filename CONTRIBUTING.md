# Contributing

感谢你参与本项目。

## 1. 开发前准备

```bash
pip install -r requirements-dev.txt
```

> 硬件相关功能（UR5、ZED、D435、Xbox）需要在真实设备环境中验证。

## 2. 分支与提交建议

- 从 `main` 拉新分支开发
- 提交信息建议使用清晰前缀（如 `feat:`、`fix:`、`docs:`、`refactor:`）

## 3. 本地检查

提交前至少执行：

```bash
python -m compileall -q src
pytest
```

## 4. 代码约定

- 保持模块职责单一，硬件控制、输入处理、数据记录分层清晰
- 新增配置优先走命令行参数，不要硬编码
- 优先使用统一日志，不再引入新的运行时 `print`
- 影响数据格式的改动必须同步更新 `README.md`

## 5. Pull Request

PR 描述建议包含改动目的、主要修改点、本地验证结果、对硬件联调的影响。
