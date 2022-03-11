#include"odom/data_frame.h"

namespace rm
{
    FrameBuffer::FrameBuffer(size_t size):
        _frames(size),
        _mutexs(size),
        _mutexs1(size),
        _tailIdx(0),
        _headIdx(0),
        _lastGetTimeStamp(0.0)
    {

    }

    bool FrameBuffer::push(const Frame& frame)
    {
        const size_t newHeadIdx = (_headIdx + 1) % _frames.size();

        //try for 2ms to lock
        std::unique_lock<std::timed_mutex> lock(_mutexs[newHeadIdx],std::chrono::milliseconds(2));
    //    unique_lock<mutex> lock(_mutexs1[newHeadIdx]);
        if(!lock.owns_lock())
        {
            return false;
        }

        _frames[newHeadIdx] = frame;
        if(newHeadIdx == _tailIdx)
        {
            _tailIdx = (_tailIdx + 1) % _frames.size();
        }
        _headIdx = newHeadIdx;
        return true;
    }

    bool FrameBuffer::getLatest(Frame& frame,bool getvid)
    {
        volatile const size_t headIdx = _headIdx;

        //try for 1ms to lock
        std::unique_lock<std::timed_mutex> lock(_mutexs[headIdx],std::chrono::milliseconds(1));
    //    unique_lock<mutex> lock(_mutexs1[headIdx]);
        if(!lock.owns_lock() ||
           _frames[headIdx].img.empty() ||
           _frames[headIdx].timeStamp == _lastGetTimeStamp)
        {
            return false;
        }

        frame = _frames[headIdx];
        if(!getvid)
        {
            _lastGetTimeStamp = _frames[headIdx].timeStamp;
        }
        return true;
    }



    Bag_FrameBuffer::Bag_FrameBuffer(size_t size)
    {
        _size=size;

    }

    bool Bag_FrameBuffer::push(const Bag_Frame& frame)
    {
        if(_Bag_Frames.size()<_size)
        {
        _Bag_Frames.push_back(frame);
        }
        else
        {
            _Bag_Frames.erase(_Bag_Frames.begin());
            _Bag_Frames.push_back(frame);
        }
        return true;

    }

    bool Bag_FrameBuffer::getLatest(Bag_Frame& frame,bool getvid)
    {
        if(_Bag_Frames.size()==0) return false;
        frame=_Bag_Frames[_Bag_Frames.size()-1];
        return true;
    }

    bool Bag_FrameBuffer::getClosest(Bag_Frame& frame,float t)
    {
        if(_Bag_Frames.size()==0) return false;
        unsigned long i=0;
        while(_Bag_Frames[i].time_stamp<t)
        {
            i++;
            if(i==_Bag_Frames.size()) break;
        }


        if(i==_Bag_Frames.size()) return false;
        else {
            if(abs(_Bag_Frames[i].time_stamp-t)<abs(_Bag_Frames[i-1].time_stamp-t))
                frame=_Bag_Frames[i];
            else {
                if(i==0) {i++;}
                frame=_Bag_Frames[i-1];
            }
            return true;
        }

    }






}



