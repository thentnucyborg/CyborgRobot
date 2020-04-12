/* ROS */
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <ros_dnn/ObjectDetectorConfig.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <ros_dnn_msgs/Prediction.h>
#include <ros_dnn_msgs/Predictions.h>
#include <ros_dnn_msgs/DetectAction.h>

/* OpenCV */
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace ros_dnn {
    class Prediction {
        public:
            Prediction(std::string label, int confidence, cv::Rect bounding_box)
                : label(label),
                  confidence(confidence),
                  bounding_box(bounding_box)
            {
            }

            /**
             * \brief Convert the prediction to a ros_dnn_msgs::Prediction message.
             */
            ros_dnn_msgs::Prediction to_prediction_msg() const;

            /**
             * \brief Get the distance from the camera to the prediction.
             */
            double get_distance(cv::Mat& depth_map) const;

            /**
             * \brief Draw a prediction on the provided frame.
             */
            void draw(cv::Mat& frame) const;

        private:
            std::string label;
            int confidence;
            cv::Rect bounding_box;
    };

    class ObjectDetectorNodelet: public nodelet::Nodelet {
        public:
            virtual void onInit();

        private:
            ros::NodeHandle nh;
            ros::NodeHandle nh_ns;

            /* Dynamic reconfigure */
            dynamic_reconfigure::Server<ros_dnn::ObjectDetectorConfig> server;
            dynamic_reconfigure::Server<ros_dnn::ObjectDetectorConfig>::CallbackType f;
            void dyn_reconf_cb(ros_dnn::ObjectDetectorConfig &config, uint32_t level);

            /* Neural network */
            cv::dnn::Net net;
            double conf_threshold;
            std::vector<std::string> class_labels;
            int frame_height;
            int frame_width;

            /* Draw a list of predictions on an image. This adds a bounding box, a label and a confidence. */
            /* TODO: Add distance */
            void draw_predictions(cv::Mat& frame, std::vector<ros_dnn::Prediction> predictions) const;

            /* Pass an image through the neural net and return a list of predictions */
            std::vector<ros_dnn::Prediction> get_predictions(cv::Mat& frame, cv::dnn::Net& net) const;

            /* Load a neural network */
            cv::dnn::Net read_network(const std::string& _model, const std::string& _config, const std::string& _framework);
            
            /* Publish/subscribe */
            image_transport::Subscriber sub_img;
            image_transport::Publisher pub_img;
            ros::Publisher pub_pred;

            void camera_cb(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& depth);
            void camera_cb(const sensor_msgs::ImageConstPtr& img);
    };
} /* Namespace ros_dnn */
