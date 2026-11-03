# SmolVLA Jetson 原生 Conda 部署 README

本文记录 HEI ReBot Lift 训练后 SmolVLA 在 NVIDIA Jetson Orin 上的原生
Conda 部署流程。目标是在 Jetson 上直接使用 CUDA 推理，不常驻 Docker，也不在
Jetson 上训练模型或解码训练数据集视频。

## 1. 已验证环境与方案边界

本文流程已经在以下环境验证：

| 项目 | 版本 |
| --- | --- |
| 架构 | `aarch64` |
| Jetson Linux | `R36.4.3` |
| JetPack 系列 | 6.2 |
| CUDA | 12.6 |
| 内存 | 8 GB 统一内存 + 11 GB swap |
| Python | 3.10（Conda） |
| PyTorch | 2.8.0，Jetson CUDA wheel |
| TorchVision | 0.23.0，Jetson CUDA wheel |
| NumPy | 1.26.4 |
| 推理策略 | SmolVLA，`sync` / `rtc` |

当前 LeRobot 源码声明 Python 3.12+。JetPack 6.2 的社区 CUDA wheel 提供
Python 3.10 版本的 PyTorch 2.8 / TorchVision 0.23，但没有与当前项目完全匹配的
Python 3.12 独立 wheel。因此本文采用以下轻量方案：

1. 创建独立 Python 3.10 Conda 环境。
2. 从 Jetson AI Lab 索引安装 CUDA 版 PyTorch 2.8 / TorchVision 0.23。
3. 复制一份 LeRobot 源码，做有限的 Python 3.10 兼容修改。
4. 实时推理不安装 TorchCodec；训练数据集仍在服务器/电脑处理。

这些兼容修改只用于 Jetson 部署副本，不应覆盖电脑端或训练服务器上的 Python
3.12 源码。若需要上游原样环境，应使用项目的 JetPack 6.2 社区 Docker 镜像；
该镜像体积较大，不属于本文方案。

## 2. 安全与资源约束

- Jetson 8 GB 内存只能用于推理，不建议训练 SmolVLA。
- 推理时关闭浏览器、Rerun、VR、桌面重应用和其他 GPU 程序。
- 同一时刻只保留一个机器人动作源。启动策略前停止遥操作、录制、回放和 VR
  真机命令发布。
- 首次执行使用 `sync`、10 FPS、10 秒，并保持急停可用。
- `swap` 可以降低 CPU 内存 OOM 概率，但不能代替 GPU/统一内存；频繁换页会显著
  增加推理延迟。
- 本文示例假设 Jetson 用户名为 `jetson`，Jetson IP 为 `192.168.31.127`，电脑
  用户名为 `lpd`。实际不同则替换对应路径/IP。

## 3. 检查 Jetson 系统

在 Jetson 执行：

```bash
uname -m
```

```bash
cat /etc/nv_tegra_release
```

```bash
nvcc --version
```

```bash
free -h
```

```bash
df -h /
```

本流程要求 `aarch64`、L4T `R36.4.x`、CUDA 12.6。Yahboom 定制系统可能没有
`nvidia-jetpack` 元包，只要 L4T/CUDA 正确，就不要为了补元包改动现有系统。

## 4. 创建独立 Conda 环境

如果 Jetson 已安装 Anaconda/Miniconda：

```bash
conda create -n smolvla python=3.10 pip -y
```

```bash
conda activate smolvla
```

```bash
python -m pip install --upgrade pip setuptools wheel
```

确认当前解释器没有指向系统 Python：

```bash
which python
```

```bash
python --version
```

预期路径类似：

```text
/home/jetson/anaconda3/envs/smolvla/bin/python
Python 3.10.x
```

安装运行库：

```bash
sudo apt update
```

```bash
sudo apt install -y libopenblas0
```

```bash
sudo apt install -y libcusparselt0
```

若系统找不到 `libcusparselt0`，不要随意添加其他 CUDA 版本的软件源；先检查
Yahboom 系统当前的 CUDA apt 源。

## 5. 安装 Jetson CUDA 版 PyTorch

只从 JetPack 6 / CUDA 12.6 索引安装 PyTorch 与 TorchVision：

```bash
python -m pip install \
  torch==2.8.0 \
  torchvision==0.23.0 \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

该 wheel 使用 NumPy 1.x 构建。立即固定 NumPy，避免导入时出现 ABI 警告：

```bash
python -m pip install --force-reinstall "numpy==1.26.4"
```

验证 GPU：

```bash
python -c "import numpy, torch; print('NumPy:', numpy.__version__); print('Torch:', torch.__version__); print('CUDA:', torch.version.cuda); print('available:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0)); print(torch.from_numpy(numpy.zeros(3, dtype=numpy.float32)).cuda())"
```

预期包含：

```text
NumPy: 1.26.4
Torch: 2.8.0
CUDA: 12.6
available: True
GPU: Orin
```

如果 `torch.cuda.is_available()` 不是 `True`，不要继续安装 LeRobot。

## 6. 创建 Jetson 专用源码副本

不要直接修改机器人 Host 正在使用的源码：

```bash
cd /home/jetson/hei-rebot-lift/software
```

```bash
cp -a lerobot-hei-rebot-lift lerobot-hei-rebot-lift-jetson
```

```bash
cd /home/jetson/hei-rebot-lift/software/lerobot-hei-rebot-lift-jetson
```

以下兼容命令只在这个副本中执行一次。

## 7. 修改 Python 与 NumPy 版本约束

允许 Python 3.10：

```bash
sed -i \
  's/requires-python = ">=3.12"/requires-python = ">=3.10"/' \
  pyproject.toml
```

固定与 Jetson PyTorch wheel 匹配的 NumPy：

```bash
sed -i \
  's/"numpy>=2.0.0,<2.3.0"/"numpy>=1.26.4,<2.0.0"/' \
  pyproject.toml
```

限制 OpenCV，防止依赖解析重新升级到 NumPy 2：

```bash
sed -i \
  's/"opencv-python-headless>=4.9.0,<4.14.0"/"opencv-python-headless>=4.9.0,<4.12.0"/' \
  pyproject.toml
```

## 8. Python 3.10 源码兼容修改

### 8.1 PEP 695 类型别名

```bash
sed -i -E \
  's/^type ([A-Za-z_][A-Za-z0-9_]*) = /\1 = /' \
  src/lerobot/datasets/aggregate.py \
  src/lerobot/motors/motors_bus.py \
  src/lerobot/rewards/sarm/sarm_utils.py
```

### 8.2 `typing.Self`

```bash
sed -i \
  's/from typing import Any, ClassVar, Self/from typing import Any, ClassVar/' \
  src/lerobot/configs/video.py
```

```bash
sed -i \
  '/from typing import Any, ClassVar/a from typing_extensions import Self' \
  src/lerobot/configs/video.py
```

```bash
sed -i \
  's/from typing import TYPE_CHECKING, Any, Self, cast/from typing import TYPE_CHECKING, Any, cast/' \
  src/lerobot/policies/wall_x/modeling_wall_x.py
```

```bash
sed -i \
  '/from typing import TYPE_CHECKING, Any, cast/a from typing_extensions import Self' \
  src/lerobot/policies/wall_x/modeling_wall_x.py
```

### 8.3 PEP 695 泛型函数与类

修复 Lance 后端：

```bash
sed -i \
  's/from typing import TYPE_CHECKING, TypedDict/from typing import TYPE_CHECKING, TypedDict, TypeVar/' \
  src/lerobot/datasets/lance_backend.py
```

```bash
sed -i \
  '/from typing import TYPE_CHECKING, TypedDict, TypeVar/a T = TypeVar("T")' \
  src/lerobot/datasets/lance_backend.py
```

```bash
sed -i 's/def _opened\[T\]/def _opened/' src/lerobot/datasets/lance_backend.py
```

修复 JSON 工具：

```bash
sed -i 's/from typing import Any/from typing import Any, TypeVar/' src/lerobot/utils/io_utils.py
```

```bash
sed -i \
  '/logger = logging.getLogger(__name__)/a T = TypeVar("T")' \
  src/lerobot/utils/io_utils.py
```

```bash
sed -i \
  's/def deserialize_json_into_object\[T: JsonLike\]/def deserialize_json_into_object/' \
  src/lerobot/utils/io_utils.py
```

修复处理器流水线：

```bash
sed -i \
  's/from typing import Any, ClassVar, TypedDict, TypeVar, cast/from typing import Any, ClassVar, Generic, TypedDict, TypeVar, cast/' \
  src/lerobot/processor/pipeline.py
```

```bash
sed -i \
  '/TOutput = TypeVar("TOutput")/a T = TypeVar("T")\nStepT = TypeVar("StepT")' \
  src/lerobot/processor/pipeline.py
```

```bash
sed -i \
  's/def register\[StepT: ProcessorStep\]/def register/' \
  src/lerobot/processor/pipeline.py
```

```bash
sed -i \
  's/def _require_component\[T\]/def _require_component/' \
  src/lerobot/processor/pipeline.py
```

```bash
sed -i \
  's/class DataProcessorPipeline\[TInput, TOutput\](HubMixin):/class DataProcessorPipeline(HubMixin, Generic[TInput, TOutput]):/' \
  src/lerobot/processor/pipeline.py
```

### 8.4 `Unpack` 与 `NotRequired`

```bash
python -m pip install --upgrade typing_extensions
```

从 Python 3.10 的 `typing` 导入中移除 `Unpack`：

```bash
sed -i -E \
  's/, Unpack//g; s/Unpack, //g' \
  src/lerobot/policies/factory.py \
  src/lerobot/policies/pretrained.py \
  src/lerobot/policies/evo1/modeling_evo1.py \
  src/lerobot/policies/dm05/modeling_dm05.py \
  src/lerobot/policies/pi0_fast/modeling_pi0_fast.py \
  src/lerobot/policies/pi05/modeling_pi05.py \
  src/lerobot/policies/pi0/modeling_pi0.py \
  src/lerobot/policies/groot/modeling_groot.py \
  src/lerobot/policies/smolvla/modeling_smolvla.py \
  src/lerobot/policies/fineart_vla/modeling_fineart_vla.py
```

```bash
sed -i \
  '/^from typing import /a from typing_extensions import Unpack' \
  src/lerobot/policies/factory.py \
  src/lerobot/policies/pretrained.py \
  src/lerobot/policies/evo1/modeling_evo1.py \
  src/lerobot/policies/dm05/modeling_dm05.py \
  src/lerobot/policies/pi0_fast/modeling_pi0_fast.py \
  src/lerobot/policies/pi05/modeling_pi05.py \
  src/lerobot/policies/pi0/modeling_pi0.py \
  src/lerobot/policies/groot/modeling_groot.py \
  src/lerobot/policies/smolvla/modeling_smolvla.py \
  src/lerobot/policies/fineart_vla/modeling_fineart_vla.py
```

修复 `NotRequired`：

```bash
sed -i -E \
  's/, NotRequired//g; s/NotRequired, //g' \
  src/lerobot/utils/transition.py \
  src/lerobot/rl/buffer.py \
  src/lerobot/datasets/aggregate.py
```

```bash
sed -i \
  '/^from typing import /a from typing_extensions import NotRequired' \
  src/lerobot/utils/transition.py \
  src/lerobot/rl/buffer.py \
  src/lerobot/datasets/aggregate.py
```

### 8.5 Python 3.10 队列类型别名

```bash
sed -i \
  '/^import logging/i from __future__ import annotations\n' \
  src/lerobot/datasets/image_writer.py
```

```bash
sed -i \
  '/from pathlib import Path/a from typing import Any' \
  src/lerobot/datasets/image_writer.py
```

```bash
sed -i \
  '/^ImageWriterQueue = ($/,/^)/c\ImageWriterQueue = Any' \
  src/lerobot/datasets/image_writer.py
```

### 8.6 兼容性检查

```bash
python -m compileall -q src/lerobot
```

没有输出表示语法检查通过。继续确认没有遗漏的 PEP 695 语法：

```bash
grep -RInE '^(class|def) [A-Za-z_][A-Za-z0-9_]*\[' src/lerobot
```

```bash
grep -RInE '^type [A-Za-z_]' src/lerobot
```

两条命令都应没有输出。

## 9. 安装 LeRobot 与 SmolVLA 依赖

保持 `smolvla` 环境激活，并位于 Jetson 专用源码根目录：

```bash
python -m pip install "opencv-python-headless==4.11.0.86"
```

```bash
python -m pip install \
  -e ".[smolvla,hei_rebot_lift]" \
  --no-build-isolation
```

`rollout.py` 当前仍会导入数据集模块，补充最小数据集依赖：

```bash
python -m pip install \
  "datasets>=4.8,<5" \
  "pandas>=2,<3" \
  "pyarrow>=21,<30" \
  "av>=15,<16" \
  "jsonlines>=4,<5"
```

不要执行：

```bash
pip install -e ".[dataset]"
```

`dataset` extra 会在 aarch64 上引入 TorchCodec 依赖；实机推理接收实时相机帧，
不需要视频数据集解码后端。

检查依赖一致性：

```bash
python -m pip check
```

如果只有原项目 Python/NumPy 声明相关问题，先确认第 7 节是否成功修改；不要让
pip 自动替换 Jetson CUDA 版 PyTorch。

## 10. 隔离用户目录 Python 包

Yahboom 系统可能在 `~/.local/lib/python3.10/site-packages` 留有旧包。它们会污染
Conda 环境，例如旧版 `soundfile` 会导致：

```text
ModuleNotFoundError: No module named '_cffi_backend'
```

当前终端先设置：

```bash
export PYTHONNOUSERSITE=1
```

确认 `sys.path` 中没有 `~/.local`：

```bash
python -c "import sys; print('\n'.join(sys.path))"
```

永久保存到当前 Conda 环境：

```bash
conda env config vars set PYTHONNOUSERSITE=1
```

```bash
conda env config vars set PYTHONPATH=/home/jetson/hei-rebot-lift/software/lerobot-hei-rebot-lift-jetson/src
```

```bash
conda deactivate
```

```bash
conda activate smolvla
```

## 11. 验证 SmolVLA 与 CUDA

```bash
python -c "import torch, torchvision; from lerobot.policies.smolvla import SmolVLAPolicy; print('torch:', torch.__version__); print('torchvision:', torchvision.__version__); print('CUDA:', torch.cuda.is_available()); print('SmolVLA OK')"
```

验证通过的输出：

```text
torch: 2.8.0
torchvision: 0.23.0
CUDA: True
SmolVLA OK
```

## 12. 复制训练权重

Jetson 创建目标目录：

```bash
mkdir -p /home/jetson/models/smolvla-task1/pretrained_model
```

在电脑端执行：

```bash
rsync -avP \
  /home/lpd/hei-rebot-lift/software/lerobot-hei-rebot-lift/outputs/smolvla-hei-rebot-lift-task1/checkpoints/010000/pretrained_model/ \
  jetson@192.168.31.127:/home/jetson/models/smolvla-task1/pretrained_model/
```

必须复制整个 `pretrained_model`，不能只复制 `model.safetensors`。Jetson 检查：

```bash
ls -lh /home/jetson/models/smolvla-task1/pretrained_model
```

至少应包含：

```text
config.json
model.safetensors
policy_preprocessor.json
policy_postprocessor.json
```

## 13. 复制 SmolVLM2 缓存并启用离线模式

该 checkpoint 的配置仍会加载：

```text
HuggingFaceTB/SmolVLM2-500M-Video-Instruct
```

因此仅复制训练 checkpoint 不够。Jetson 创建缓存目录：

```bash
mkdir -p /home/jetson/huggingface/hub
```

电脑端执行（`rsync -a` 会保留缓存中的相对符号链接）：

```bash
rsync -avP \
  /home/lpd/.cache/huggingface/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct/ \
  jetson@192.168.31.127:/home/jetson/huggingface/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct/
```

在 Jetson 保存离线环境变量：

```bash
conda env config vars set HF_HOME=/home/jetson/huggingface
```

```bash
conda env config vars set HF_HUB_OFFLINE=1
```

```bash
conda env config vars set TRANSFORMERS_OFFLINE=1
```

```bash
conda deactivate
```

```bash
conda activate smolvla
```

## 14. 不连接机器人，只加载模型

在一个终端观察内存：

```bash
tegrastats
```

在另一个终端执行：

```bash
cd /home/jetson/hei-rebot-lift/software/lerobot-hei-rebot-lift-jetson
```

```bash
conda activate smolvla
```

```bash
python -c "from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy; policy = SmolVLAPolicy.from_pretrained('/home/jetson/models/smolvla-task1/pretrained_model'); print('Checkpoint loaded on:', policy.config.device)"
```

若提示离线缓存不存在，检查 `HF_HOME` 和第 13 节的模型目录；不要关闭离线模式后
反复联网重试。

## 15. 启动机器人 Host

Host 继续使用原有、未经 Python 3.10 修改的机器人环境。在 Jetson 终端 1 执行：

```bash
cd /home/jetson/hei-rebot-lift/software/lerobot-hei-rebot-lift
```

```bash
conda activate lerobot5
```

```bash
export PYTHONPATH=/home/jetson/hei-rebot-lift/software/lerobot-hei-rebot-lift/src
```

```bash
hei-rebot-lift-host
```

等待升降回零完成。不要在两个环境中同时启动 Host。

## 16. 首次实机推理

Jetson 终端 2 执行：

```bash
cd /home/jetson/hei-rebot-lift/software/lerobot-hei-rebot-lift-jetson
```

```bash
conda activate smolvla
```

首次只运行 10 秒同步推理：

```bash
python -u examples/hei_rebot_lift/rollout.py \
  --remote-ip 127.0.0.1 \
  --model-id /home/jetson/models/smolvla-task1/pretrained_model \
  --task "Grasp a small plush toy and place it into an orange box" \
  --duration-sec 10 \
  --fps 10 \
  --inference sync
```

确认相机、关节方向、动作范围和急停都正常后，再使用 RTC：

```bash
python -u examples/hei_rebot_lift/rollout.py \
  --remote-ip 127.0.0.1 \
  --model-id /home/jetson/models/smolvla-task1/pretrained_model \
  --task "Grasp a small plush toy and place it into an orange box" \
  --duration-sec 60 \
  --fps 10 \
  --inference rtc
```

不要添加 `--display-data`。当前项目的 `rollout.py` 若未显式初始化 Rerun，会在
控制循环开始时因缺少 application id 退出。

## 17. 常见问题

### 17.1 NumPy 2 ABI 警告

症状：

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x
```

修复：

```bash
python -m pip install --force-reinstall "numpy==1.26.4"
```

并确认 `pyproject.toml` 已将 NumPy 上限改为 `<2.0.0`。

### 17.2 `Unpack` / `NotRequired` 导入错误

症状：

```text
ImportError: cannot import name 'Unpack' from 'typing'
```

原因是 Python 3.10 需要从 `typing_extensions` 导入。重新执行第 8.4 节并检查：

```bash
grep -RInE '^from typing import .*Unpack|^from typing import .*NotRequired' src/lerobot
```

### 17.3 `queue.Queue[...]` 不可下标

症状：

```text
TypeError: 'type' object is not subscriptable
```

确认：

```bash
grep -n "ImageWriterQueue" src/lerobot/datasets/image_writer.py
```

应显示：

```text
ImageWriterQueue = Any
```

### 17.4 `soundfile` / `_cffi_backend` 错误

若错误路径位于 `~/.local/lib/python3.10/site-packages`，说明用户目录包污染了 Conda：

```bash
export PYTHONNOUSERSITE=1
```

不要优先在 Conda 环境中堆叠安装音频包；SmolVLA 图像/文本推理不需要音频。

### 17.5 离线模式找不到 SmolVLM2

检查：

```bash
echo $HF_HOME
```

```bash
find /home/jetson/huggingface/hub -maxdepth 2 -type d -name 'models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct'
```

checkpoint 中的 `vlm_model_name` 是 Hugging Face repo id，因此缓存必须保持标准
Hugging Face Hub 目录结构。

### 17.6 Jetson OOM 或严重卡顿

1. 停止桌面浏览器、Rerun、VR和其他CUDA任务。
2. 使用 `--fps 10`，不要直接从30 FPS开始。
3. 用 `tegrastats`观察RAM、swap、GPU和温度。
4. 先验证`sync`，再启用RTC后台推理线程。
5. 若模型加载阶段已耗尽内存，不要依赖增加swap解决GPU统一内存瓶颈。

## 18. 清理由中断Docker下载产生的缓存

本文方案不需要Docker。如果之前中断过大镜像下载，可先检查：

```bash
sudo docker image ls
```

```bash
sudo docker system df
```

只清理悬空镜像和构建缓存：

```bash
sudo docker image prune -f
```

```bash
sudo docker builder prune -f
```

不要直接使用 `docker system prune -a`，它可能删除机器人已有的 ROS、标注或其他
项目镜像。

## 19. 最终验收清单

- [ ] Jetson 是 `aarch64`、L4T R36.4.x、CUDA 12.6。
- [ ] `torch==2.8.0`、`torchvision==0.23.0`。
- [ ] `torch.cuda.is_available()` 为 `True`。
- [ ] NumPy 固定为 1.26.4，没有 ABI 警告。
- [ ] `python -m compileall -q src/lerobot` 无输出。
- [ ] `SmolVLAPolicy` 导入成功。
- [ ] 完整 `pretrained_model` 已复制。
- [ ] SmolVLM2 Hub 缓存已复制，离线加载成功。
- [ ] Host 与策略是唯一两个必要进程，没有其他动作源。
- [ ] 首次真机推理使用 10 秒、10 FPS、`sync`。
