# X-CUBE-AI-Implementation-of-MobileNetv1-using-STM32CubeIDE

# MobileNetv1
The model used in this repository is:

- Width multiplier: 0.25
- Image Resolution: 160
- Pretrained MobileNetv1: [Tensorflow](https://github.com/tensorflow/models/blob/master/research/slim/nets/mobilenet_v1.md)
- Quantization: 8-bit 
- Paper: [link](https://arxiv.org/abs/1704.04861)

# Requirements
The project is made on the following versions:

- STMCubeIDE 1.8.0
- X-CUBE-AI 7.1.0
- Board: NUCLEO STM32H743ZIT6U

Check the versions before implementing, and if you encounter any error please move to the above-mentioned versions. While, the board can be anyone that supports the x-cube-ai library. The official documentation for x-cube-ai is [here](https://www.st.com/resource/en/user_manual/dm00570145-getting-started-with-x-cube-ai-expansion-package-for-artificial-intelligence-ai-stmicroelectronics.pdf). 

# About Project
The project contains the STM32CubeIDE project template files. The files description is as follows:

- **Core**:  Contains the source and include files necessary for the project execution.
- **IOC file**: "Mobv1_160_0.25_XCUBEAI.ioc" contains the project configurations. 
- **Middlewares**: A generated code by the project configurations in .ioc file for the Mobilenet.
- **Validation results**: Validation results from the project.
- **TensorFlow Lite model**: "mobilenet_v1_0.25_160_quant.tflite" imagenet quantized model.

The implementation of other versions of MobileNetv1 will be coming soon. 


