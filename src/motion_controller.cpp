#include <gazebo/gazebo.hh>//主要这个头文件，包含 Gazebo 中的大部分对象
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
//ROS 要用到的所用到的头文件
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
//ROS 的消息文件
#include <std_msgs/Float32.h>
//所有 Gazebo 插件都要在 Gazebo 的命名空间下创建。Gazebo 是之前头文件中已有的命名空间。
namespace gazebo
{
  /// \brief A plugin to control car motion.
  //可以把插件看作是C++中的类
  class PositionPlugin : public ModelPlugin//定义插件的类名，该类继承 ModelPlugin
  {
    /// \brief Constructor 类对象构造函数，在gazebo中创建该插件时会执行一次
    public: PositionPlugin() 
    {//一些测试代码，显示插件已经被构建了
      std::cout<<"Motion Plugin"<<std::endl;
    }

    /// \brief 在该插件被加载进模型中时，gazebo会运行一次 Load 函数
    /// \param[in] _model A pointer to the model that this plugin is　attached to.
    ///  _model 指针参数会指向该插件所在的模型对象
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    /// _sdf指针指向插件对应的sdf元素。这种一般是指向world文件中编写的插件的 SDF 代码。
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      std::cout<<"Starting Load"<<std::endl;
      this->model = _model;
      std::cerr << "\n The model's name is [" <<
        _model->GetScopedName() << "]\n";

      std::string car_name_ori="Hello";//不好意思，这里把名字取的太随意了，可以自行修改（T T）

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, car_name_ori + "_" + "node",
            ros::init_options::NoSigintHandler);
      }

      // 根据之前定义的名字，创建ROS节点
      this->rosNode.reset(new ros::NodeHandle(car_name_ori + "_" + "Handle"));

      // 创建一个ＲＯＳ中的ｔｏｐｉｃ
      ros::SubscribeOptions so =
      //create 函数创造使用相应信息数据类型的ｔｏｐｉｃ
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/" + car_name_ori + "/pos_cmd",//ｔｏｐｉｃ的名称
            1,//ｔｏｐｉｃ在通道内保存信息的数量个数
            boost::bind(&PositionPlugin::OnRosMsg, this, _1),//该ｔｏｐｉｃ对应的回调函数
            ros::VoidPtr(), &this->rosQueue);//该ｔｏｐｉｃ的数据队列是用户自定义的，所以要将自己创建的 CallbackQueue 对象传递过去(引用传递)
      this->rosSub = this->rosNode->subscribe(so);
      //看到这里朋友们可以发现，该话题的创建方式和 ROS 教程中不同。上述这种方式最主要的点在于用户要自行给定topic的队列控制函数，即rosQueue。用户要在函数中控制topic多久处理一次队列中的信息。

      //将控制topic队列的函数放进新的线程中，这样才能使topic正常工作。
      this->rosQueueThread =
        std::thread(std::bind(&PositionPlugin::QueueThread, this));

	//将设置模型速度的函数和仿真世界刷新函数绑定，这样才能有效改变模型速度（不断试错发现的）
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&PositionPlugin::OnUpdate, this));
    }
    
    /// \brief Handle an incoming message from ROS上述topic对应的回调函数
    /// \param[in] _msg A float value that is used to control the car
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      //std::cout<<"I can hear you "<<std::endl;
      this->vel=_msg->data;
      sleep(1);//加入这个会让小车只运动一秒
      this->vel=0;
    }

    public:void OnUpdate()
        {	
            this->model->SetLinearVel(ignition::math::Vector3d(this->vel, 0, 0));//设置模型的线速度
            //this->model->SetAngularVel(ignition::math::Vector3d(this->vel, 0, 0));//设置模型的角速度
            //设置速度的函数要和仿真世界刷新函数共同执行，放在ROS回调函数中无效。
        }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()//这个函数是每多少timeout时间处理一次存储在ｔｏｐｉｃ通道中的信息。这个函数和之前的创建topic的代码要配套使用，否则topic会无法工作
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
        //因为ｔｏｐｉｃ创建的方式比较特殊，故要设置数据队列处理数据的时间间隔！！！！！！
      }
    }
    
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    private: double vel;//速度变量
    private: event::ConnectionPtr updateConnection;
    
  };
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin. 
  GZ_REGISTER_MODEL_PLUGIN(PositionPlugin)//这部分一定要有，这样才能让 gazebo 知道这是个插件
} // namespace gazebo
