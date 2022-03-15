# 说明
## 节点的定义

定义my_point类

具有x  y    self    father域

self和father是存储了这个对象在memory_pool中的下标

此代码使用A*算法，每次将待搜索的节点new出来，进入内存池后加入优先队列，优先级根据估值函数h(x)给出，这一部分也是急需完善的部分，也是最重要的一部分

搜索完毕后把内存池全都delete掉避免内存浪费

这份代码写的是把优先队列里面的点全都delete掉，这样事实上有问题，为什么呢，因为有的节点已经被pop出队了，你把优先队列里面的删光还有些废物节点没删，这个改一下就行了，下一版会修正之

2021/10/21 之前说的问题已经修复了，但是目前使用的A*加上火力代价地图之后陷入了局部贪心，不过其实问题不大，只要不被打啥都好说，绕一绕就绕一绕呗

2021/10/22 现在遇到了很多问题，第一个是添加斜对角扩展之后，路径规划速度一下暴涨到平均18ms？？？我不能理解！第二个是路径规划中整体走的是直线，但是小格子上面是曲折的，这样我怎么判断转弯？？这个有待讨论！不过今天重新设计了架构，可以很好的适配仿真函数、导航函数以及为放入最后的工程做了铺垫。而且今天大概设计了一下导航的方法。虽然没写，不过觉得多少有点问题。这个拐点的玩意给我整不会了。。。

2021/11/10 连续上两周网课，上的我真就直接摆烂，不过上次开会给的任务很重，这两天连夜肝出来了这一份路径规划好使的代码，不过耗时已经达到了150ms(最长情况下)，代码也变得极其臃肿，不过至少跑起来了。我和老队长说，明天必然拍出一个像样的测试视频，他说，那要是拍不出来呢？我只能说，必定做出来，不可能做不出来！另外，在gitee上面写这些与代码无关的故事似乎会被封，今后考虑在zhihu上面记录一下开发过程，真的是很宝贵的经历啊。