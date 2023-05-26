# digit_recognizer 手写数字识别器

**这不是 ROS 功能包。**

在部署到小车上之前，需要先在小车上安装 `ncnn` 和 `OpenCV` 库。

创建文件夹 `build`, 进入 `build` 文件夹，执行 `cmake ..` 与 `make`.

执行以下命令，链接到模型文件：

```bash
ln -s ../model/yolox.bin yolox.bin
ln -s ../model/yolox.param yolox.param
```

运行 `./digit_recognizer_demo`, 小车将从摄像头 0 读取视频流并进行实时识别。

也可以运行 `./digit_recognizer_demo [path2image]`, 识别单张图片。
