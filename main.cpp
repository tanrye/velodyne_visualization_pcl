#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
///#include <pcd_io.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>

using namespace boost;
using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleVLPViewer
{
public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    /*pcl::Grbber是pcl对应的设备驱动接口的基类定义--采集卡*/
    /* 这里Cloud是定义了*/
    /* 这里CloudConstPtr只做了声明为Cloud::ConstPtr类型,后面可以用来用在不同场合的变量，而具有同一类型*/
    SimpleVLPViewer (pcl::Grabber& grabber,pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)  //这里给Grabber和获取RGB的类分别引用一个别名，等同操作
        : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL VLP Cloud")),    /*cloud_viewer_是指针变量*/
       grabber_ (grabber),
       handler_ (handler)
    {
    }

    void cloud_callback (const CloudConstPtr& cloud)    /*定义cloud为Cloud::ConstPtr型变量*/
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void run ()
    {
        //cloud_viewer_对显示窗口内容的设置,非对点云数据本身的处理
      cloud_viewer_->addCoordinateSystem (0.5);           //这里添加3D坐标系，用于显示点云的尺寸，这里轴设为默认单位的3倍，默认值是1，
      cloud_viewer_->setBackgroundColor (0,0,0);       //设置视窗的背景色，这里设为黑色，可以设任意rgb
      cloud_viewer_->initCameraParameters ();          //初始化传感器参数，使得从需要的方向和角度观察点云
      cloud_viewer_->setCameraPosition (2.0,2.0,0.0,0.0,0.0,0.0,0);   //设置坐标原点pos,up,viewport ?
      cloud_viewer_->setCameraClipDistances (0.0,100.0,1);    //near,far,viewport ?

      function<void (const CloudConstPtr&)> cloud_cb = bind (&SimpleVLPViewer::cloud_callback, this, _1);
      signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);     //缓冲回调??
      grabber_.start ();            //是对能传输信息流的设备，使能传输  开始传输    重要！！！,后面就处理从这里输入的信息！！！！
      while (!cloud_viewer_->wasStopped ())               //|停止显示
      {
        CloudConstPtr cloud;                              //pointXYZ lei 的对象

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())        //锁住该线程防止从其他进程写入点云
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();            //解锁上面的lock
        }/**/

        if (cloud)
        {
          handler_.setInputCloud (cloud);                                   //将cloud点云数据送到窗口显示，这里是只要xyz型
          if (!cloud_viewer_->updatePointCloud (cloud, handler_, "VLP"))   //updatePointCloud()提供需要的点云ID，实时刷新
            cloud_viewer_->addPointCloud (cloud, handler_, "VLP");         //实现点云的添加,更新,id实时在变
            //addPointCloud(source,sources_cloud_color,"sources_cloud_v1",v1); //将点云source,处理结果sources_cloud_color,添加到视图中，其中,双引号中的sources_cloud_v1,表示该点云视窗的名字，之所以设置各个处理点云的名字，是为了在后续处理中易于区分; v1表是添加到哪个视图窗口（pcl中可设置多窗口模式）
          cloud_viewer_->spinOnce ();                                      //目的是一直显示
          //pcl::io::savePCDFileASCII ("write_pcd_test2.pcd", *cloud);
       //   std::cout<<"FPS "<<grabber_.getFramesPerSecond()<<std::endl;

          std::cout << "PointCloud sum: " << cloud->points.size () << " data points."
                    <<" weight "<<cloud->width<<" height "<<cloud->height<< std::endl;

        }

        if (!grabber_.isRunning ())     //采集口没有数据流传输时，当设备没有传入pcl信息（没有工作时）时，停止显示
          cloud_viewer_->spin ();
          //grabber_.getFramesPerSecond();
        this_thread::sleep (posix_time::microseconds (10));

     }
      grabber_.stop ();                  //停止传输

      cloud_connection.disconnect ();

    }

    shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};

int main (int argc, char ** argv)
{
  //string hdlCalibration, pcapFile;
  //typedef VLPGrabber HDLGrabber;

  // parse_argument (argc, argv, "-calibrationFile", hdlCalibration);                /*该函数解析特定的命令行参数，前三个参数是输入量，calibrationFile是要搜索的字符串值，hdlCalibration是输出值*/
  // parse_argument (argc, argv, "-pcapFile", pcapFile);

  //VLPGrabber* grabber=new VLPGrabber (pcapFile);                                                    /*hdlCalibration,用于捕获雷达数据的函数，监听IP端口，该函数第一个参数是校准参数即反射强度信息，是第二个参数是捕获的数据包（都可以认为是其包的文件）*/
   VLPGrabber grabber;  //grabber(pcapFile)
   //std::cout<<"FPS "<<grabber.getFramesPerSecond()<<std::endl;

  PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");  /*这里是对输入为xyzi类型的点云着色，这里颜色基于强度信息的改变，并用于VTK可视化*/
 // PointCloudColorHandlerGenericField<pcl::PointWithRange> color_handler ("range");
   SimpleVLPViewer v (grabber,color_handler);
  /*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
  //std::cout << rangeImage << "\n";
  /*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
                                         /*simpleVLPViewer的对象才是想看到的，而其传入参数grabber便是需要处理的数据*/
  v.run ();

  //pcl::io::savePCDFileASCII ("write_pcd_test.pcd", Cloud); 　　　/*对于VLP显示时，画面隔一段时间会闪烁下，可能是刷新频率比传输速率要快，这里拟用先保存，在一段延时后在读取数据显示*/

  return (0);
}

/*-------------------------------------小记---------------------------------------------------------------------------------------------------------------------------*/
/*PCL1.8增加了对VLP16的解析库VLPGrabber,增加了VLP16的信息，同时私有成员继承了之前HDL的类，1.8兼容了1.7所以本程序中使用HDL解析VLP16也是可以的，这里处理的是HDL的双返回模式，16雷达一般也是设置的是双返回模式*/
/*目前该显示可以直接用于显示处理后的点云以观察特征，后期在这个基础上升级成纯正的VLP16的可视化程序*/
//这里改成VLP会出现点云闪烁、不稳定，不改形成32线，与实际不符，存在一条线一部分是检测到平面成直线而还有部分“自己”成曲线，暂不确定这样对后面的目标是否有影响
//用vlpgrabber还是一闪一闪的，老子不搞这个，还是用hdl32先用着，应当不会对检测产生影响，而且在可视化看到的线束增多，明显
