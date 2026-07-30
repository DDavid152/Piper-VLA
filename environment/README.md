# 环境定义目录

本目录保存 `lerobot` 环境的最小定义、兼容约束和当前机器的解析快照。安装
逻辑位于 `scripts/`，不要手工修改生成型锁文件。

## 文件

| 文件 | 作用 |
|---|---|
| `environment.yml` | 最小 Conda 定义：Python 3.12、FFmpeg 7、Pip |
| `constraints.txt` | Torch、TorchCodec、PyAV、Piper、CAN、Orbbec 关键版本 |
| `conda-explicit-linux-64.txt` | 当前 Linux x86_64 Conda 包的精确 URL |
| `environment.resolved.yml` | 不含 build 字段的完整 Conda/Pip 解析结果 |
| `requirements.lock.txt` | 完整 Pip 版本及三个本地 editable 插件路径 |
| `pip-inspect.json` | Pip 元数据、依赖关系和安装来源审计 |
| `activate.d/piper-vla.sh` | 激活时将 Conda FFmpeg 运行库置于搜索路径前部 |
| `deactivate.d/piper-vla.sh` | 退出环境时恢复原 `LD_LIBRARY_PATH` |

`pip-inspect.json` 虽然较大，但用于依赖来源审计，不能当作缓存删除。

## 使用

激活：

```bash
source /home/ubuntu22/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
```

从零安装：

```bash
./scripts/install_miniforge.sh
pkexec ./scripts/install_system_dependencies.sh
./scripts/install_python_environment.sh
pkexec ./scripts/install_orbbec_udev.sh
./scripts/verify_environment.sh
./scripts/freeze_environment.sh
```

环境版本变化后运行 `./scripts/freeze_environment.sh`，并同时提交四个生成型
清单。不要只更新 `constraints.txt` 而遗漏锁文件。
