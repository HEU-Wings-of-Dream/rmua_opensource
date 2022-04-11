/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#pragma once
#include<opencv2/opencv.hpp>

class RMVideoCapture
{
public:
    RMVideoCapture();
    RMVideoCapture(const int id, int buffer_size = 2);
	~RMVideoCapture();
    //打开摄像头
    bool open(const int id, int size_buffer = 2);
    //判断摄像头是否打开
    bool isOpened() const;

//    void setBufferSize(size_t bsize);
    //设置分辨率
    bool setVideoFormat(size_t width, size_t height, bool mjpg = 1);
    //设置曝光
    bool setExposureTime(int t = 0);
    //改变分辨率
    bool changeVideoFormat(int width, int height, bool mjpg = 1);
    //设置帧率
    bool setFPS(int fps);
    //开始捕获视频流
	bool startStream();
    //结束捕获视频流
	bool closeStream();
    //重新打开摄像头
	void restartCapture();

	RMVideoCapture& operator >> (cv::Mat & image);

    bool read(cv::Mat& image);

    /*
     * @Brief:  Grabs the next frame from video file or capturing device.
     * @Return: 'true' if success
     */
    //从摄像头获取下一帧数据
    bool grab();

    /*
     * @Brief:  Decodes and returns the grabbed video frame.
     * @Output: img:    the video frame is returned here. If no frames has been grabbed
     *                  the image remains unchanged.
     */
    //解码图片并返回
    bool retrieve(cv::Mat& image);
    //获得分辨率数据
	cv::Size getResolution();
    //获得帧编号
	int getFrameCount()
	{
        return _frameCount;
	}
    //输出摄像头信息
	void info();


private:
	struct MapBuffer
	{
		void * ptr;
		unsigned int length;
	};
    //分辨率
    unsigned int _width;
    unsigned int _height;
    unsigned int _format;

    int _camera_fd;
    bool _is_opened;
    //缓存大小
    unsigned int _buffer_size;
    unsigned int _bufferIdx;
    unsigned int _frameCount;
    MapBuffer* _buffers;
    //摄像头文件路径
    std::string _videoPath;

    bool _grabbed_left;


    //void cvtRaw2Mat(void * data,unsigned int length, cv::Mat & image);
    //摄像头数据转换为Mat
    void cvtRaw2Mat(void * data, cv::Mat & image);
    //刷新获得当前摄像头参数 分辨率和格式
    bool refreshVideoFormat();
    //初始化摄像头缓存
    bool initMMap();
    //对摄像头进行读写操作
    int xioctl(int fd, int request, void *arg);
};

