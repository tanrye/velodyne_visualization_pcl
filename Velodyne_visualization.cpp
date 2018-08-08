#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
//#include <pcd_io.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>

//using namespace boost;
using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;
//using boost::shared_ptr;

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
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);//shu yu ku wen jian de cuo wu
      grabber_.start ();            //是对能传输信息流的设备
      while (!cloud_viewer_->wasStopped ())               //|停止显示
      {
        CloudConstPtr cloud;                              //pointXYZ lei 的对象

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())        //锁住该线程防止从其他进程写入点云    //缓冲回调??
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
        boost::this_thread::sleep (boost::posix_time::microseconds (10));

     }
      grabber_.stop ();                  //停止传输

      cloud_connection.disconnect ();

    }
//private:
    shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};

int main (int argc, char ** argv)
{
   VLPGrabber grabber;  //grabber(pcapFile)
   //std::cout<<"FPS "<<grabber.getFramesPerSecond()<<std::endl;

   PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");  /*这里是对输入为xyzi类型的点云着色，这里颜色基于强度信息的改变，并用于VTK可视化*/
   SimpleVLPViewer v (grabber,color_handler);
                                        /*simpleVLPViewer的对象才是想看到的，而其传入参数grabber便是需要处理的数据*/
  v.run ();

  //pcl::io::savePCDFileASCII ("write_pcd_test.pcd", Cloud); 　　　/*对于VLP显示时，画面隔一段时间会闪烁下，可能是刷新频率比传输速率要快，这里拟用先保存，在一段延时后在读取数据显示*/

  return (0);
}
