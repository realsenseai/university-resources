# Track 2: RealSense + OpenVINO Mastery

## 🎯 Learning Objectives

By the end of this track, you will be able to:
- Optimize AI models for Intel hardware using OpenVINO
- Build high-performance RGB-D inference pipelines
- Deploy real-time 3D perception on edge devices
- Create custom OpenVINO operations for depth processing
- Achieve production-grade performance optimization

## 🧠 OpenVINO Fundamentals

### What is OpenVINO?

**OpenVINO** (Open Visual Inference and Neural Network Optimization) is Intel's toolkit for optimizing and deploying AI models on Intel hardware:

- **Model Optimization**: Quantization, pruning, and graph optimization
- **Hardware Acceleration**: CPU, GPU, VPU, and FPGA support
- **Cross-Platform**: Linux, Windows, macOS deployment
- **Framework Support**: PyTorch, TensorFlow, ONNX model import

### OpenVINO Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Training Framework                       │
│              (PyTorch, TensorFlow, ONNX)                    │
└─────────────────────────┬───────────────────────────────────┘
                          │ Export
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    Model Optimizer                           │
│         (Conversion, Optimization, Quantization)            │
└─────────────────────────┬───────────────────────────────────┘
                          │ IR Format (.xml + .bin)
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                   Inference Engine                           │
│              (Runtime, Plugin Architecture)                 │
└─────────────────────────┬───────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        ▼                 ▼                 ▼
   ┌─────────┐       ┌─────────┐       ┌─────────┐
   │   CPU   │       │   GPU   │       │   VPU   │
   │ Plugin  │       │ Plugin  │       │ Plugin  │
   └─────────┘       └─────────┘       └─────────┘
```

## 🔧 OpenVINO Setup

### Installation

```bash
# Install OpenVINO toolkit
pip install openvino openvino-dev

# Install additional tools
pip install nncf  # Neural Network Compression Framework
pip install onnx onnxruntime

# Verify installation
python -c "from openvino import Core; print(Core().available_devices)"
```

### Basic Inference Pipeline

```python
from openvino import Core, CompiledModel
from openvino.preprocess import PrePostProcessor
import numpy as np
import cv2

class OpenVINOInference:
    def __init__(self, model_path: str, device: str = "AUTO"):
        self.core = Core()
        self.device = device
        
        # Load model
        self.model = self.core.read_model(model_path)
        
        # Configure preprocessing
        self.setup_preprocessing()
        
        # Compile for target device
        self.compiled_model = self.core.compile_model(
            self.model, device
        )
        
        # Create inference request
        self.infer_request = self.compiled_model.create_infer_request()
        
    def setup_preprocessing(self):
        """Configure model preprocessing"""
        ppp = PrePostProcessor(self.model)
        
        # Input configuration
        ppp.input().tensor() \
            .set_element_type(np.uint8) \
            .set_layout("NHWC") \
            .set_color_format(ColorFormat.BGR)
        
        ppp.input().preprocess() \
            .convert_element_type(np.float32) \
            .convert_color(ColorFormat.RGB) \
            .scale([255.0])
        
        ppp.input().model().set_layout("NCHW")
        
        self.model = ppp.build()
    
    def infer(self, input_data: np.ndarray) -> dict:
        """Run inference"""
        # Set input
        self.infer_request.set_input_tensor(
            Tensor(input_data)
        )
        
        # Run inference
        self.infer_request.infer()
        
        # Get outputs
        outputs = {}
        for output in self.compiled_model.outputs:
            outputs[output.any_name] = self.infer_request.get_output_tensor(
                output.index
            ).data.copy()
        
        return outputs
    
    def infer_async(self, input_data: np.ndarray, callback=None):
        """Run asynchronous inference"""
        self.infer_request.set_input_tensor(Tensor(input_data))
        
        if callback:
            self.infer_request.set_callback(callback)
        
        self.infer_request.start_async()
    
    def wait(self):
        """Wait for async inference to complete"""
        self.infer_request.wait()
```

## 🎯 RGB-D Model Optimization

### Converting PyTorch Models

```python
import torch
import openvino as ov
from openvino.tools import mo

class RGBDModelConverter:
    def __init__(self):
        self.core = ov.Core()
        
    def convert_pytorch_model(self, model: torch.nn.Module,
                              input_shapes: dict,
                              output_path: str):
        """Convert PyTorch model to OpenVINO IR"""
        model.eval()
        
        # Create dummy inputs
        dummy_inputs = {}
        for name, shape in input_shapes.items():
            dummy_inputs[name] = torch.randn(shape)
        
        # Export to ONNX
        onnx_path = output_path.replace('.xml', '.onnx')
        torch.onnx.export(
            model,
            tuple(dummy_inputs.values()),
            onnx_path,
            input_names=list(input_shapes.keys()),
            output_names=['output'],
            dynamic_axes={name: {0: 'batch'} for name in input_shapes.keys()},
            opset_version=11
        )
        
        # Convert to OpenVINO IR
        ov_model = ov.convert_model(onnx_path)
        ov.save_model(ov_model, output_path)
        
        return output_path
    
    def convert_rgbd_detector(self, rgb_model, depth_model, output_path):
        """Convert RGB-D detection model"""
        # Create combined model
        class RGBDDetector(torch.nn.Module):
            def __init__(self, rgb_backbone, depth_backbone, fusion_head):
                super().__init__()
                self.rgb_backbone = rgb_backbone
                self.depth_backbone = depth_backbone
                self.fusion_head = fusion_head
                
            def forward(self, rgb, depth):
                rgb_features = self.rgb_backbone(rgb)
                depth_features = self.depth_backbone(depth)
                fused = torch.cat([rgb_features, depth_features], dim=1)
                return self.fusion_head(fused)
        
        combined = RGBDDetector(rgb_model, depth_model, fusion_head)
        
        input_shapes = {
            'rgb': [1, 3, 480, 640],
            'depth': [1, 1, 480, 640]
        }
        
        return self.convert_pytorch_model(combined, input_shapes, output_path)
```

### Model Quantization

```python
import nncf
from nncf import NNCFConfig
from nncf.torch import create_compressed_model
import torch
from torch.utils.data import DataLoader

class ModelQuantizer:
    def __init__(self):
        self.quantization_config = {
            "input_info": [
                {"sample_size": [1, 3, 480, 640]},
                {"sample_size": [1, 1, 480, 640]}
            ],
            "compression": {
                "algorithm": "quantization",
                "preset": "mixed",
                "overflow_fix": "disable",
                "initializer": {
                    "range": {"num_init_samples": 300},
                    "batchnorm_adaptation": {"num_bn_adaptation_samples": 300}
                }
            }
        }
        
    def quantize_model(self, model: torch.nn.Module,
                       calibration_dataloader: DataLoader,
                       output_path: str):
        """Quantize model using NNCF"""
        model.eval()
        
        # Create NNCF config
        nncf_config = NNCFConfig.from_dict(self.quantization_config)
        
        # Create compressed model
        compression_ctrl, compressed_model = create_compressed_model(
            model, nncf_config
        )
        
        # Calibrate on sample data
        compressed_model.eval()
        with torch.no_grad():
            for batch_idx, (rgb, depth, _) in enumerate(calibration_dataloader):
                if batch_idx >= 300:
                    break
                compressed_model(rgb, depth)
        
        # Export to ONNX
        onnx_path = output_path.replace('.xml', '.onnx')
        compression_ctrl.export_model(onnx_path)
        
        # Convert to OpenVINO
        ov_model = ov.convert_model(onnx_path)
        ov.save_model(ov_model, output_path)
        
        return output_path
    
    def quantize_to_int8(self, model_path: str, 
                        calibration_data: list,
                        output_path: str):
        """Post-training INT8 quantization"""
        import openvino as ov
        from openvino.tools import pot
        
        # Load model
        core = ov.Core()
        model = core.read_model(model_path)
        
        # Define data loader for calibration
        class CalibrationDataLoader:
            def __init__(self, data):
                self.data = data
                self.index = 0
                
            def __iter__(self):
                return self
            
            def __next__(self):
                if self.index >= len(self.data):
                    raise StopIteration
                item = self.data[self.index]
                self.index += 1
                return item
            
            def __len__(self):
                return len(self.data)
        
        # Configure quantization
        algorithms = [
            {
                "name": "DefaultQuantization",
                "params": {
                    "target_device": "CPU",
                    "preset": "mixed",
                    "stat_subset_size": 300
                }
            }
        ]
        
        engine_config = {
            "device": "CPU",
            "stat_requests_number": 2
        }
        
        # Run quantization
        data_loader = CalibrationDataLoader(calibration_data)
        
        compressed_model = pot.compress_model_weights(
            model, 
            mode=pot.CompressWeightsMode.INT8_ASYM
        )
        
        ov.save_model(compressed_model, output_path)
        return output_path
```

## 🚀 Real-Time RGB-D Inference

### High-Performance Pipeline

```python
import pyrealsense2 as rs
import numpy as np
import cv2
from openvino import Core, AsyncInferQueue
from collections import deque
import threading
import time

class RealTimeRGBDInference:
    def __init__(self, model_path: str, 
                 device: str = "AUTO",
                 num_requests: int = 4):
        self.core = Core()
        self.device = device
        
        # Load and compile model
        self.model = self.core.read_model(model_path)
        
        # Optimize for latency
        config = {
            "PERFORMANCE_HINT": "LATENCY",
            "NUM_STREAMS": "1"
        }
        
        self.compiled_model = self.core.compile_model(
            self.model, device, config
        )
        
        # Create async inference queue
        self.infer_queue = AsyncInferQueue(
            self.compiled_model, num_requests
        )
        self.infer_queue.set_callback(self._inference_callback)
        
        # Results buffer
        self.results = deque(maxlen=100)
        self.frame_count = 0
        
        # Performance metrics
        self.latencies = deque(maxlen=100)
        self.start_times = {}
        
    def _inference_callback(self, infer_request, userdata):
        """Callback for completed inference"""
        frame_id = userdata
        
        # Calculate latency
        latency = time.time() - self.start_times.pop(frame_id, time.time())
        self.latencies.append(latency)
        
        # Get results
        outputs = {}
        for output in self.compiled_model.outputs:
            outputs[output.any_name] = infer_request.get_output_tensor(
                output.index
            ).data.copy()
        
        self.results.append({
            'frame_id': frame_id,
            'outputs': outputs,
            'latency': latency
        })
    
    def infer_async(self, rgb: np.ndarray, depth: np.ndarray):
        """Submit frame for async inference"""
        frame_id = self.frame_count
        self.frame_count += 1
        
        # Record start time
        self.start_times[frame_id] = time.time()
        
        # Preprocess
        rgb_input = self._preprocess_rgb(rgb)
        depth_input = self._preprocess_depth(depth)
        
        # Submit to queue
        self.infer_queue.start_async(
            {
                self.model.inputs[0]: rgb_input,
                self.model.inputs[1]: depth_input
            },
            userdata=frame_id
        )
        
        return frame_id
    
    def _preprocess_rgb(self, rgb: np.ndarray) -> np.ndarray:
        """Preprocess RGB image"""
        # Resize if needed
        rgb = cv2.resize(rgb, (640, 480))
        
        # Normalize and transpose
        rgb = rgb.astype(np.float32) / 255.0
        rgb = np.transpose(rgb, (2, 0, 1))
        rgb = np.expand_dims(rgb, 0)
        
        return rgb
    
    def _preprocess_depth(self, depth: np.ndarray) -> np.ndarray:
        """Preprocess depth image"""
        # Resize if needed
        depth = cv2.resize(depth, (640, 480))
        
        # Normalize depth (assuming 16-bit mm values)
        depth = depth.astype(np.float32) / 10000.0
        depth = np.expand_dims(depth, (0, 1))
        
        return depth
    
    def get_result(self, frame_id: int, timeout: float = 0.1):
        """Get result for specific frame"""
        start = time.time()
        while time.time() - start < timeout:
            for result in self.results:
                if result['frame_id'] == frame_id:
                    return result
            time.sleep(0.001)
        return None
    
    def get_latest_result(self):
        """Get most recent result"""
        if self.results:
            return self.results[-1]
        return None
    
    def get_performance_stats(self):
        """Get performance statistics"""
        if not self.latencies:
            return {}
        
        latencies = list(self.latencies)
        return {
            'avg_latency_ms': np.mean(latencies) * 1000,
            'min_latency_ms': np.min(latencies) * 1000,
            'max_latency_ms': np.max(latencies) * 1000,
            'fps': 1.0 / np.mean(latencies) if np.mean(latencies) > 0 else 0,
            'pending_requests': len(self.start_times)
        }
    
    def wait_all(self):
        """Wait for all pending inferences"""
        self.infer_queue.wait_all()


class RealSenseOpenVINOSystem:
    def __init__(self, model_path: str, device: str = "AUTO"):
        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Initialize inference
        self.inference = RealTimeRGBDInference(model_path, device)
        
        # Align depth to color
        self.align = rs.align(rs.stream.color)
        
        self.running = False
        
    def start(self):
        """Start the system"""
        self.profile = self.pipeline.start(self.config)
        self.running = True
        
        # Start processing thread
        self.process_thread = threading.Thread(target=self._process_loop)
        self.process_thread.start()
    
    def _process_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                # Get frames
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                aligned_frames = self.align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # Convert to numpy
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # Submit for inference
                self.inference.infer_async(color_image, depth_image)
                
            except Exception as e:
                print(f"Error in processing loop: {e}")
    
    def get_detections(self):
        """Get latest detections"""
        result = self.inference.get_latest_result()
        if result:
            return self._parse_detections(result['outputs'])
        return []
    
    def _parse_detections(self, outputs):
        """Parse model outputs to detections"""
        # Implementation depends on model architecture
        detections = []
        
        if 'boxes' in outputs and 'scores' in outputs:
            boxes = outputs['boxes']
            scores = outputs['scores']
            classes = outputs.get('classes', np.zeros_like(scores))
            
            for i in range(len(scores)):
                if scores[i] > 0.5:
                    detections.append({
                        'box': boxes[i],
                        'score': scores[i],
                        'class': int(classes[i])
                    })
        
        return detections
    
    def stop(self):
        """Stop the system"""
        self.running = False
        self.process_thread.join()
        self.inference.wait_all()
        self.pipeline.stop()
```

## 📱 Edge Deployment

### NVIDIA Jetson Optimization

```python
import subprocess
import os

class JetsonOptimizer:
    def __init__(self):
        self.power_modes = {
            'MAX': 0,
            '30W': 1,
            '15W': 2
        }
        
    def set_power_mode(self, mode: str):
        """Set Jetson power mode"""
        if mode in self.power_modes:
            subprocess.run([
                'sudo', 'nvpmodel', '-m', str(self.power_modes[mode])
            ])
    
    def enable_max_clocks(self):
        """Enable maximum clock speeds"""
        subprocess.run(['sudo', 'jetson_clocks'])
    
    def get_system_stats(self):
        """Get Jetson system statistics"""
        result = subprocess.run(
            ['tegrastats', '--interval', '100'],
            capture_output=True,
            timeout=1
        )
        return result.stdout.decode()


class EdgeInferenceOptimizer:
    def __init__(self, model_path: str):
        from openvino import Core
        self.core = Core()
        self.model = self.core.read_model(model_path)
        
    def optimize_for_edge(self, target_device: str = "CPU"):
        """Optimize model for edge deployment"""
        # Configure for low latency
        config = {
            "PERFORMANCE_HINT": "LATENCY",
            "INFERENCE_NUM_THREADS": "4",
            "AFFINITY": "CORE"
        }
        
        if target_device == "GPU":
            config.update({
                "GPU_THROUGHPUT_STREAMS": "1",
                "GPU_QUEUE_PRIORITY": "HIGH"
            })
        
        return self.core.compile_model(self.model, target_device, config)
    
    def benchmark(self, compiled_model, num_iterations: int = 100):
        """Benchmark inference performance"""
        import time
        
        # Create dummy input
        input_shape = list(self.model.inputs[0].shape)
        input_shape[0] = 1
        dummy_input = np.random.randn(*input_shape).astype(np.float32)
        
        # Warm up
        infer_request = compiled_model.create_infer_request()
        for _ in range(10):
            infer_request.infer({0: dummy_input})
        
        # Benchmark
        times = []
        for _ in range(num_iterations):
            start = time.perf_counter()
            infer_request.infer({0: dummy_input})
            times.append(time.perf_counter() - start)
        
        return {
            'avg_ms': np.mean(times) * 1000,
            'std_ms': np.std(times) * 1000,
            'min_ms': np.min(times) * 1000,
            'max_ms': np.max(times) * 1000,
            'fps': 1.0 / np.mean(times)
        }
```

### Intel Neural Compute Stick (NCS2) Support

```python
from openvino import Core

class NCS2Inference:
    def __init__(self, model_path: str):
        self.core = Core()
        
        # Check for MYRIAD device
        devices = self.core.available_devices
        if 'MYRIAD' not in devices:
            raise RuntimeError("NCS2 not found. Available devices: " + str(devices))
        
        # Load and compile for MYRIAD
        self.model = self.core.read_model(model_path)
        
        # MYRIAD-specific configuration
        config = {
            "MYRIAD_THROUGHPUT_STREAMS": "1",
            "MYRIAD_ENABLE_HW_ACCELERATION": "YES"
        }
        
        self.compiled_model = self.core.compile_model(
            self.model, "MYRIAD", config
        )
        
        self.infer_request = self.compiled_model.create_infer_request()
    
    def infer(self, input_data: np.ndarray) -> dict:
        """Run inference on NCS2"""
        # NCS2 requires FP16
        input_fp16 = input_data.astype(np.float16)
        
        self.infer_request.infer({0: input_fp16})
        
        outputs = {}
        for i, output in enumerate(self.compiled_model.outputs):
            output_data = self.infer_request.get_output_tensor(i).data
            outputs[output.any_name] = output_data.astype(np.float32)
        
        return outputs
```

## 🔬 Custom OpenVINO Extensions

### Depth Processing Operation

```python
import numpy as np
from openvino import Core, Model, PartialShape
from openvino.runtime import opset10 as opset
from openvino.runtime.passes import Manager

class DepthNormalizationOp:
    """Custom operation for depth normalization"""
    
    @staticmethod
    def create_model(min_depth: float = 0.1, max_depth: float = 10.0):
        """Create depth normalization subgraph"""
        # Input: depth image [N, 1, H, W]
        depth_input = opset.parameter(
            PartialShape([-1, 1, -1, -1]),
            np.float32,
            name="depth"
        )
        
        # Clamp to valid range
        min_const = opset.constant(np.array([min_depth]), np.float32)
        max_const = opset.constant(np.array([max_depth]), np.float32)
        
        clamped = opset.clamp(depth_input, min_const, max_const)
        
        # Normalize to [0, 1]
        range_val = opset.constant(np.array([max_depth - min_depth]), np.float32)
        shifted = opset.subtract(clamped, min_const)
        normalized = opset.divide(shifted, range_val)
        
        # Create result
        result = opset.result(normalized, name="normalized_depth")
        
        return Model([result], [depth_input], "depth_normalization")


class PointCloudGeneratorOp:
    """Custom operation for point cloud generation"""
    
    @staticmethod
    def create_model(fx: float, fy: float, cx: float, cy: float):
        """Create point cloud generation subgraph"""
        # Input: depth image [N, 1, H, W]
        depth_input = opset.parameter(
            PartialShape([1, 1, 480, 640]),
            np.float32,
            name="depth"
        )
        
        # Create coordinate grids
        u_coords = np.arange(640, dtype=np.float32).reshape(1, 1, 1, 640)
        v_coords = np.arange(480, dtype=np.float32).reshape(1, 1, 480, 1)
        
        u_const = opset.constant(np.broadcast_to(u_coords, (1, 1, 480, 640)))
        v_const = opset.constant(np.broadcast_to(v_coords, (1, 1, 480, 640)))
        
        # Camera intrinsics
        fx_const = opset.constant(np.array([fx]), np.float32)
        fy_const = opset.constant(np.array([fy]), np.float32)
        cx_const = opset.constant(np.array([cx]), np.float32)
        cy_const = opset.constant(np.array([cy]), np.float32)
        
        # Calculate X = (u - cx) * depth / fx
        u_shifted = opset.subtract(u_const, cx_const)
        x = opset.multiply(u_shifted, depth_input)
        x = opset.divide(x, fx_const)
        
        # Calculate Y = (v - cy) * depth / fy
        v_shifted = opset.subtract(v_const, cy_const)
        y = opset.multiply(v_shifted, depth_input)
        y = opset.divide(y, fy_const)
        
        # Stack to create point cloud [N, 3, H, W]
        points = opset.concat([x, y, depth_input], axis=1)
        
        result = opset.result(points, name="point_cloud")
        
        return Model([result], [depth_input], "point_cloud_generator")
```

## 🧪 Hands-On Exercises

### Exercise 1: Model Conversion
1. Train a simple RGB-D detector in PyTorch
2. Export to ONNX format
3. Convert to OpenVINO IR
4. Compare inference speed

### Exercise 2: Quantization
1. Collect calibration dataset
2. Apply INT8 quantization
3. Measure accuracy vs speed tradeoff
4. Optimize for specific hardware

### Exercise 3: Real-Time Pipeline
1. Build async inference pipeline
2. Integrate with RealSense
3. Achieve 30+ FPS detection
4. Minimize latency

### Exercise 4: Edge Deployment
1. Deploy on NVIDIA Jetson
2. Deploy on Intel NCS2
3. Compare performance
4. Optimize for each platform

## 📊 Performance Benchmarks

### Reference Results

| Model | Platform | Precision | Latency | FPS |
|-------|----------|-----------|---------|-----|
| YOLOv8n | Intel i7-12700 | FP32 | 8.5ms | 117 |
| YOLOv8n | Intel i7-12700 | INT8 | 4.2ms | 238 |
| YOLOv8n | Intel Arc A770 | FP16 | 3.1ms | 322 |
| YOLOv8n | NCS2 | FP16 | 45ms | 22 |
| YOLOv8n | Jetson Orin | FP16 | 5.8ms | 172 |

## 🎯 Next Steps

Ready to continue? → [Track 3: RealSense Developer SDK Extensions](./track-3-sdk-extensions.md)
