#include "ros_dnn/object_detector_nodelet.hpp"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_dnn::ObjectDetectorNodelet, nodelet::Nodelet);

using namespace std;

namespace ros_dnn {
    ros_dnn_msgs::Prediction Prediction::to_prediction_msg() const
    {
        geometry_msgs::Point32 pt;
        ros_dnn_msgs::Prediction msg;

        msg.label = label;
        msg.confidence = confidence;
        msg.distance = 0;

        /* Top left */
        pt.x = bounding_box.tl().x;
        pt.y = bounding_box.tl().y;
        msg.bounding_box.points.push_back(pt);

        /* Top right */
        pt.x = bounding_box.br().x;
        pt.y = bounding_box.tl().y;
        msg.bounding_box.points.push_back(pt);

        /* Bottom right */
        pt.x = bounding_box.br().x;
        pt.y = bounding_box.br().y;
        msg.bounding_box.points.push_back(pt);

        /* Bottom left */
        pt.x = bounding_box.tl().x;
        pt.y = bounding_box.br().y;
        msg.bounding_box.points.push_back(pt);

        return msg;
    }

    double Prediction::get_distance(cv::Mat& depth_map) const
    {
        cv::Mat image_roi = depth_map(bounding_box);
        cv::Scalar mean_depth = cv::mean(image_roi);

        return mean_depth[0];
    }

    void Prediction::draw(cv::Mat& frame) const
    {
        /* Draw class label */
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::rectangle(frame,
                      cv::Point(bounding_box.tl().x, bounding_box.tl().y - labelSize.height),
                      cv::Point(bounding_box.tl().x + labelSize.width, bounding_box.tl().y + baseLine),
                      cv::Scalar::all(255),
                      cv::FILLED);

        cv::putText(frame,
                    label,
                    bounding_box.tl(),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar());

        /* Draw bounding box */
        cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0));
    }

    void ObjectDetectorNodelet::onInit()
    {
        bool use_depth;

        std::string net_model;
        std::string net_config;
        std::string net_framework;
    
        std::string camera_topic_name;
        int camera_topic_qsize;

        std::string predictions_topic_name;
        int predictions_topic_qsize;
        bool predictions_topic_latch;

        std::string detection_image_topic_name;
        int detection_image_topic_qsize;
        bool detection_image_topic_latch;

        nh = getMTNodeHandle();
        nh_ns = getMTPrivateNodeHandle();

        /* Get parameters from launch file */
        nh_ns.getParam("model_path", net_model);
        nh_ns.getParam("config_path", net_config);
        nh_ns.getParam("framework", net_framework);
        nh_ns.getParam("frame_height", frame_height);
        nh_ns.getParam("frame_width", frame_width);
        nh_ns.getParam("class_labels", class_labels);

        nh_ns.getParam("camera_topic_name", camera_topic_name);
        nh_ns.getParam("camera_topic_qsize", camera_topic_qsize);

        use_depth = nh_ns.getParam("depth_topic_name", camera_topic_name);
        nh_ns.getParam("depth_topic_qsize", camera_topic_qsize);

        nh_ns.getParam("predictions_topic_name", predictions_topic_name);
        nh_ns.getParam("predictions_topic_qsize", predictions_topic_qsize);
        nh_ns.getParam("predictions_topic_latch", predictions_topic_latch);

        nh_ns.getParam("detection_image_topic_name", detection_image_topic_name);
        nh_ns.getParam("detection_image_topic_qsize", detection_image_topic_qsize);
        nh_ns.getParam("detection_image_topic_latch", detection_image_topic_latch);

        /* Create subscribers and subscribers */
        image_transport::ImageTransport it(nh);

        /* Subscribe to left color image */
        image_transport::SubscriberFilter sub_rgb(it, camera_topic_name, camera_topic_qsize);
        NODELET_INFO_STREAM("Subscribed to topic " << camera_topic_name);

        if (use_depth) {
            /* Subscribe to depth map */
            image_transport::SubscriberFilter sub_depth(it, depth_topic_name, depth_topic_qsize);
            NODELET_INFO_STREAM("Subscribed to topic " << depth_topic_name);

            typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
            const ExactSyncPolicy sync_policy(std::min(camera_topic_qsize, depth_topic_qsize));

            message_filters::Synchronizer<ExactSyncPolicy> sync(sync_policy, sub_rgb, sub_depth);
            sync.registerCallback(boost::bind(&camera_cb, _1, _2));

        } else {
            message_filters::Synchronizer<ExactSyncPolicy> sync(sync_policy, sub_rgb);
            sync.registerCallback(boost::bind(&camera_cb, _1));
        }

        /* Create prediction publisher */
        pub_pred = nh.advertise<ros_dnn_msgs::Predictions>(predictions_topic_name, predictions_topic_qsize, predictions_topic_latch);
        NODELET_INFO_STREAM("Advertising on topic " << predictions_topic_name);

        /* Create detection image publisher */
        pub_img = it.advertise(detection_image_topic_name, detection_image_topic_qsize, detection_image_topic_latch);
        NODELET_INFO_STREAM("Advertising on topic " << detection_image_topic_name);

        /* Initialize neural network */
        net = read_network(net_model, net_config, net_framework);

        /* Initialize dynamic reconfigure */
        f = boost::bind(&ObjectDetectorNodelet::dyn_reconf_cb, this, _1, _2);
        server.setCallback(f);
    }

    /* This is cv::dnn::readNet from OpenCV 3.4, and should be removed once ROS upgrades to that version. */
    cv::dnn::Net ObjectDetectorNodelet::read_network(const std::string& _model, const std::string& _config, const std::string& _framework)
    {
        std::string framework = _framework;
        std::string model = _model;
        std::string config = _config;
        const std::string modelExt = model.substr(model.rfind('.') + 1);
        const std::string configExt = config.substr(config.rfind('.') + 1);
        if (framework == "caffe" || modelExt == "caffemodel" || configExt == "caffemodel" ||
                                    modelExt == "prototxt" || configExt == "prototxt")
        {
            if (modelExt == "prototxt" || configExt == "caffemodel")
                std::swap(model, config);
            return cv::dnn::readNetFromCaffe(config, model);
        }
        if (framework == "tensorflow" || modelExt == "pb" || configExt == "pb" ||
                                        modelExt == "pbtxt" || configExt == "pbtxt")
        {
            if (modelExt == "pbtxt" || configExt == "pb")
                std::swap(model, config);
            return cv::dnn::readNetFromTensorflow(model, config);
        }
        if (framework == "torch" || modelExt == "t7" || modelExt == "net" ||
                                    configExt == "t7" || configExt == "net")
        {
            return cv::dnn::readNetFromTorch(model.empty() ? config : model);
        }
        if (framework == "darknet" || modelExt == "weights" || configExt == "weights" ||
                                    modelExt == "cfg" || configExt == "cfg")
        {
            if (modelExt == "cfg" || configExt == "weights")
                std::swap(model, config);
            return cv::dnn::readNetFromDarknet(config, model);
        }
        NODELET_INFO("Network initialization failed.");
    }

    void ObjectDetectorNodelet::draw_predictions(cv::Mat& frame, std::vector<ros_dnn::Prediction> predictions) const
    {
        for (const auto& prediction : predictions) {
            prediction.draw(frame);
        }
    }

    std::vector<ros_dnn::Prediction> ObjectDetectorNodelet::get_predictions(cv::Mat& frame, cv::dnn::Net& net) const
    {
        static std::vector<int> outLayers = net.getUnconnectedOutLayers();
        static std::string outLayerType = net.getLayer(outLayers[0])->type;

        cv::Mat blob, out;
        std::vector<ros_dnn::Prediction> predictions;

        cv::Size frame_size(frame_width > 0 ? frame_width : frame.cols, frame_height > 0 ? frame_height : frame.rows);
        blob = cv::dnn::blobFromImage(frame, 1 / 255.F, frame_size, true, false);
        net.setInput(blob);

        /* Faster-RCNN or R-FCN */
        if (net.getLayer(0)->outputNameToIndex("im_info") != -1)
        {
            resize(frame, frame, frame_size);
            cv::Mat imInfo = (cv::Mat_<float>(1, 3) << frame_size.height, frame_size.width, 1.6f);
            net.setInput(imInfo, "im_info");
        }

        out = net.forward();

        float* data = (float*)out.data;
        if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
        {
            // Network produces output blob with a shape 1x1xNx7 where N is a number of
            // detections and an every detection is a vector of values
            // [batchId, class_id, confidence, left, top, right, bottom]
            for (size_t i = 0; i < out.total(); i += 7)
            {
                float confidence = data[i + 2];
                if (confidence > conf_threshold)
                {
                    int left = (int)data[i + 3];
                    int top = (int)data[i + 4];
                    int right = (int)data[i + 5];
                    int bottom = (int)data[i + 6];
                    int class_id = (int)(data[i + 1]) - 1;  // Skip 0th background class id.

                    std::string label = cv::format("%.2f", confidence);
                    if (!class_labels.empty())
                    {
                        assert(class_id < (int)class_labels.size());
                        label = class_labels[class_id] + ": " + label;
                    }

                    cv::Point top_left = cv::Point(left, top);
                    cv::Point bottom_right = cv::Point(right, bottom);

                    predictions.push_back(
                            ros_dnn::Prediction(
                                label,
                                confidence,
                                cv::Rect(cv::Point(left, top), cv::Point(right, bottom))));
                }
            }
        }
        else if (outLayerType == "DetectionOutput")
        {
            // Network produces output blob with a shape 1x1xNx7 where N is a number of
            // detections and an every detection is a vector of values
            // [batchId, classId, confidence, left, top, right, bottom]
            for (size_t i = 0; i < out.total(); i += 7)
            {
                float confidence = data[i + 2];
                if (confidence > conf_threshold)
                {
                    int left = (int)(data[i + 3] * frame.cols);
                    int top = (int)(data[i + 4] * frame.rows);
                    int right = (int)(data[i + 5] * frame.cols);
                    int bottom = (int)(data[i + 6] * frame.rows);
                    int class_id = (int)(data[i + 1]) - 1;  // Skip 0th background class id.

                    std::string label = cv::format("%.2f", confidence);
                    if (!class_labels.empty())
                    {
                        assert(class_id < (int)class_labels.size());
                        label = class_labels[class_id] + ": " + label;
                    }

                    predictions.push_back(
                            ros_dnn::Prediction(
                                label,
                                confidence,
                                cv::Rect(cv::Point(left, top), cv::Point(right, bottom))));
                }
            }
        }
        else if (outLayerType == "Region")
        {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            for (int i = 0; i < out.rows; ++i, data += out.cols)
            {
                cv::Mat confidences = out.row(i).colRange(5, out.cols);
                cv::Point classIdPoint;
                double confidence;
                cv::minMaxLoc(confidences, 0, &confidence, 0, &classIdPoint);
                if (confidence > conf_threshold)
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    int class_id = classIdPoint.x;

                    std::string label = cv::format("%.2f", confidence);
                    if (!class_labels.empty())
                    {
                        assert(class_id < (int)class_labels.size());
                        label = class_labels[class_id] + ": " + label;
                    }

                    predictions.push_back(
                            ros_dnn::Prediction(
                                label,
                                confidence,
                                cv::Rect(cv::Point(left, top), cv::Point(left+width, top+height))));
                }
            }
        }
        else
            NODELET_ERROR("Unknown output layer type: %s", outLayerType.c_str());

        return predictions;
    }

    void ObjectDetectorNodelet::camera_cb(const sensor_msgs::ImageConstPtr& rgb)
    {
        cv::Mat frame;
        sensor_msgs::ImagePtr out_msg;

        /* Abort if no one is subscribed */
        if (pub_img.getNumSubscribers() == 0) {
            return;
        }

        ros::Time begin = msg->header.stamp;
        ros::Time now = ros::Time::now();
        ros::Duration diff = now - begin;

        NODELET_INFO("%ld", diff.toNSec());

        /* Transform image to OpenCV format */
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            NODELET_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /* Get predictions and draw them on the frame */
        std::vector<ros_dnn::Prediction> predictions = get_predictions(frame, net);
        draw_predictions(frame, predictions);

        /* Return the new image */
        out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub_img.publish(out_msg);

        /* Return the predictions */
    }

    void ObjectDetectorNodelet::camera_cb(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth)
    {
        NODELET_INFO("Processing image with depth");
    }

    void ObjectDetectorNodelet::dyn_reconf_cb(ros_dnn::ObjectDetectorConfig &config, uint32_t level)
    {
        NODELET_INFO("Reconfigure confidence threshold : %f", config.conf_threshold);
        conf_threshold = config.conf_threshold;
        NODELET_INFO("Threshold: %f", conf_threshold);
    }
} /* Namespace ros_dnn */
