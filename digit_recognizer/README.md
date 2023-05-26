# digit_recognizer 手写数字识别器

**这不是 ROS 功能包。**

在部署到小车上之前，需要先在小车上安装 `ncnn` 和 `OpenCV` 库。

创建文件夹 `build`, 进入 `build` 文件夹，执行 `cmake ..` 与 `make`.

将 `yolox.param` 与 `yolox.bin` 放置在 `digit_recognizer_demo` 可执行文件的相同目录下，运行 `./digit_recognizer_demo`, 小车将从摄像头 0 读取视频流并进行实时识别。
