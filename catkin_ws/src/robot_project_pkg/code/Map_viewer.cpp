#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"


// Struttura per memorizzare le coordinate del mouse e altri dettagli
struct ClickData {
    cv::Point map_coords;  // Coordinate sulla mappa
    cv::Point opencv_coords;  // Coordinate nel sistema OpenCV
    ros::Time click_time;  // Tempo del click
};
enum ClickMode {
    SELECT_INITIAL_POSITION,
    SELECT_GOAL_POSITION
};
class MapViewer {
public:
    MapViewer() {
        
        map_sub = nh.subscribe("/map", 1, &MapViewer::mapCallback, this);
        scan_sub = nh.subscribe("/base_scan", 1, &MapViewer::scanCallback, this);
        odom_sub = nh.subscribe("/odom", 1, &MapViewer::odomCallback, this);
        obstacle_sub = nh.subscribe("/obstacle_map", 1, &MapViewer::obstacleCallback, this);
         initial_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        goal_publisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
        cv::namedWindow("Map", cv::WINDOW_NORMAL);
        cv::setMouseCallback("Map", &MapViewer::mouseCallback, this);
        map_initialized = false;
        odom_initialized = false;

// Inizializza le posizioni a valori non validi
        initial_position = ClickData{cv::Point(-99999, -99999), cv::Point(-99999, -99999), ros::Time(0)};
        goal_position = ClickData{cv::Point(-99999, -99999), cv::Point(-99999, -99999), ros::Time(0)};
        click_mode = SELECT_INITIAL_POSITION;

    }

    bool isMapInitialized() const {
        return map_initialized && odom_initialized;
    }
     // Funzione che si occupa di pubblicare le posizioni su /initialpose e /move_base/goal
    void publishPosition() {
    // Controlla se le coordinate sono valide
    bool is_initial_position_valid = initial_position.map_coords.x != -99999 && initial_position.map_coords.y != -99999;
    bool is_goal_position_valid = goal_position.map_coords.x != -99999 && goal_position.map_coords.y != -99999;

    if (is_initial_position_valid) {
        // Pubblica la posizione iniziale
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = -initial_position.map_coords.x;
        pose_msg.pose.pose.position.y = -initial_position.map_coords.y;
        //pose_msg.pose.pose.position.x = -(0+106.913*0.02)/2;
        //pose_msg.pose.pose.position.y = -(0+49.3527 *0.02)/2;
        pose_msg.pose.pose.position.z = 0;  // Nessuna componente Z, solo posizione 2D
        pose_msg.pose.pose.orientation.w = 1.0;
        
         
        initial_publisher.publish(pose_msg);
    }

    if (is_goal_position_valid) {
        // Pubblica il goal
        move_base_msgs::MoveBaseActionGoal goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";
        goal_msg.goal.target_pose.header.stamp = ros::Time::now();
        goal_msg.goal.target_pose.header.frame_id = "map";  // Frame di riferimento del goal
        goal_msg.goal.target_pose.pose.position.x = -goal_position.map_coords.x;
        goal_msg.goal.target_pose.pose.position.y = -goal_position.map_coords.y;
        //goal_msg.goal.target_pose.pose.position.x = 40;
        //goal_msg.goal.target_pose.pose.position.y = 12;
        goal_msg.goal.target_pose.pose.position.z = 0;
        goal_msg.goal.target_pose.pose.orientation.w = 1.0;

        goal_publisher.publish(goal_msg);
    }
}


private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber obstacle_sub;
    ros::Publisher initial_publisher;  // Publisher per la posizione iniziale
    ros::Publisher goal_publisher;  // Publisher per la posizione goal
    bool map_initialized;
    bool odom_initialized;
    nav_msgs::OccupancyGridConstPtr map_msg;
    sensor_msgs::LaserScanConstPtr scan_msg;
    nav_msgs::OdometryConstPtr odom_msg;
    nav_msgs::OccupancyGridConstPtr obstacle_msg;

    ClickData initial_position;  // Posizione iniziale
    ClickData goal_position;  // Posizione goal
    ClickMode click_mode;  // Modalità di click
    // Conversione coordinate Stage in OpenCV
    cv::Point stageToOpenCV(float x_stage, float y_stage, int width_cv, int height_cv) {
        int x_opencv = static_cast<int>(x_stage * 20 + width_cv / 2);
        int y_opencv = static_cast<int>(y_stage * 20 + height_cv / 2);
        return cv::Point(x_opencv, y_opencv);
    
    }

   cv::Point opencvToStage(float x_open, float y_open, int map_width, int map_height, float resolution, float origin_x, float origin_y) {
        float stage_x = (x_open - map_width / 2) * resolution + origin_x;
        float stage_y = (y_open - map_height / 2) * resolution + origin_y;  // Inverti Y
        return cv::Point(stage_x, stage_y);
    }
    


    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        map_msg = msg;
        map_initialized = true;
    }
    static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        MapViewer* viewer = reinterpret_cast<MapViewer*>(userdata);
        
        if (!viewer->map_msg) {
            ROS_WARN("Map not yet initialized.");
            return;
        }

        int map_width = viewer->map_msg->info.width;
        int map_height = viewer->map_msg->info.height;
        float resolution = viewer->map_msg->info.resolution;
        float origin_x = viewer->map_msg->info.origin.position.x;
        float origin_y = viewer->map_msg->info.origin.position.y;

        cv::Point opencv_coords(x, y);
        cv::Point map_coords = viewer->opencvToStage(x, y, map_width, map_height, resolution, origin_x, origin_y);
        cv::Point map_coords_adjusted(x, y);
            map_coords_adjusted.x=(map_coords.x+106.913*0.02)/2;
            map_coords_adjusted.y=(map_coords.y+49.3527 *0.02)/2;
            

        if (viewer->click_mode == SELECT_INITIAL_POSITION) {
            viewer->initial_position = {map_coords_adjusted, opencv_coords, ros::Time::now()};  // Memorizza le coordinate iniziali
            viewer->click_mode = SELECT_GOAL_POSITION;  // Cambia modalità
            // Chiama la funzione per pubblicare la posizione iniziale
            
        } else if (viewer->click_mode == SELECT_GOAL_POSITION) {
            viewer->goal_position = {map_coords_adjusted, opencv_coords, ros::Time::now()};  // Memorizza le coordinate goal
            // Chiama la funzione per pubblicare la posizione del goal
            viewer->publishPosition();
            viewer->click_mode = SELECT_INITIAL_POSITION;
        }
        
    }
}


    void scanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
        scan_msg = msg;
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        odom_msg = msg;
        odom_initialized = true;
    }

    void obstacleCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        obstacle_msg = msg;
    }

    void drawMapWithScans() {
        if (!isMapInitialized()) {
            ROS_WARN("La mappa o l'odometria non sono inizializzate.");
            return;
        }

        int width = map_msg->info.width;
        int height = map_msg->info.height;
        float resolution = map_msg->info.resolution;
        float origin_x = map_msg->info.origin.position.x;
        float origin_y = map_msg->info.origin.position.y;

        // Get the robot's yaw orientation
        tf2::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Creazione dell'immagine della mappa
        cv::Mat map_image(height, width, CV_8UC3, cv::Scalar(255, 255, 255)); 

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int value = map_msg->data[index];
                cv::Vec3b color;
                if (value == -1) {
                    color = cv::Vec3b(128, 128, 128); // Grigio per sconosciuto
                } else if (value == 0) {
                    color = cv::Vec3b(255, 255, 255); // Bianco per area libera
                } else {
                    color = cv::Vec3b(0, 0, 255); // Rosso per area occupata
                }
                 map_image.at<cv::Vec3b>(cv::Point(x, y)) = color;
            }
        }
        
        // Disegnare la scansione del laser
        if (scan_msg) {
            std::vector<cv::Point> scan_points;
            float robot_x = odom_msg->pose.pose.position.x;
            float robot_y = odom_msg->pose.pose.position.y;

            for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
                float range = scan_msg->ranges[i];
                if (std::isinf(range) || std::isnan(range)) {
                    continue; // Saltare valori infiniti o NaN
                }

                float angle = scan_msg->angle_min + i * scan_msg->angle_increment + yaw;
                float x_scan = robot_x + range * std::cos(angle);
                float y_scan = robot_y + range * std::sin(angle);

                cv::Point cv_point = stageToOpenCV(x_scan, y_scan, width, height);

                if (cv_point.x >= 0 && cv_point.x < width && cv_point.y >= 0 && cv_point.y < height) {
                    scan_points.push_back(cv_point);
                }
            }

            if (!scan_points.empty()) {
            // Aggiungi il punto del robot all'inizio e alla fine per chiudere il poligono
            cv::Point robot_point = stageToOpenCV(robot_x, robot_y, width, height);
            scan_points.insert(scan_points.begin(), robot_point); // Aggiungi all'inizio
            scan_points.push_back(robot_point); // Aggiungi alla fine

            // Crea il poligono con i punti della scansione
            const std::vector<std::vector<cv::Point>> polygon = {scan_points};

            // Riempi il poligono con un colore specifico (verde)
            cv::fillPoly(map_image, polygon, cv::Scalar(0, 255, 0));
        }
            
        }
        // Disegna il robot
        cv::Point robot_pos = stageToOpenCV(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, width, height);
        int robot_x = origin_x + robot_pos.x;
        int robot_y = origin_y + robot_pos.y;

        cv::circle(map_image, cv::Point(robot_x, robot_y), 5, cv::Scalar(0, 0, 255), -1);
        cv::flip(map_image, map_image, 0);
        // Disegna gli ostacoli
        if (obstacle_msg) {
            int obstacle_width = obstacle_msg->info.width;
            int obstacle_height = obstacle_msg->info.height;
            float obstacle_resolution = obstacle_msg->info.resolution;

            for (int y = 0; y < obstacle_height; ++y) {
                for (int x = 0; x < obstacle_width; ++x) {
                    int index = y * obstacle_width + x;
                    int value = obstacle_msg->data[index];

                    if (value == -1) {
                        continue; // Celle sconosciute
                    }

                    float x_stage = (x * obstacle_resolution) + obstacle_msg-> info.origin.position.x;
                    float y_stage = (y * obstacle_resolution) + obstacle_msg-> info.origin.position.y;

                    cv::Point obstacle_pos = stageToOpenCV(x_stage, y_stage, width, height);
                    if (obstacle_pos.x >= 0 && obstacle_pos.x < width && obstacle_pos.y >= 0 && obstacle_pos.y < height) {
                        map_image.at<cv::Vec3b>(obstacle_pos) = cv::Vec3b(255, 0, 0); // Rosso per ostacoli
                    }
                }
            }
        }

        

        // Box informativo in alto a sinistra
        int box_x = 10;
        int box_y = 10;
        int box_width = 500;
        int box_height = 60;
        //conversioni
        // Converti yaw in gradi per semplificare la visualizzazione
        double yaw_degrees = yaw * (180.0 / M_PI); // Converti da radianti a gradi
        // Ottieni la coordinata Z del robot
        double robot_z = odom_msg->pose.pose.position.z;
        std::string degree_symbol = "\u00B0";  // Simbolo del grado in Unicode
        cv::rectangle(map_image, cv::Point(box_x, box_y), cv::Point(box_x + box_width, box_y + box_height), cv::Scalar(0, 255, 0), 2);
        
        std::string current_position_text = "Robot Position: x=" + std::to_string(robot_x) +
                                    ", y=" + std::to_string(robot_y) +
                                    ", z=" + std::to_string(robot_z) +
                                    ", yaw=" + std::to_string(yaw_degrees) + degree_symbol;
        cv::putText(map_image, current_position_text, cv::Point(box_x + 10, box_y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        // Box informativo per l'initial position
        std::string initial_position_text = "Initial Position: ";
        if (ros::Time::now() - initial_position.click_time < ros::Duration(10.0)) {
        initial_position_text += "x=" + std::to_string(initial_position.opencv_coords.x) +
                             ", y=" + std::to_string(initial_position.opencv_coords.y);
        } else {
        initial_position_text += "No click detected.";
        }

        // Disegna il box per l'initial position
        int initial_box_x = 10;
        int initial_box_y = box_y + 140; // Posizionato sotto il box del goal
        cv::rectangle(map_image, cv::Point(initial_box_x, initial_box_y), cv::Point(initial_box_x + box_width, initial_box_y + box_height), cv::Scalar(0, 255, 255), 2);
        cv::putText(map_image, initial_position_text, cv::Point(initial_box_x + 10, initial_box_y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

        // Box informativo per il goal
        std::string goal_text = "Goal: ";
        if (ros::Time::now() - goal_position.click_time < ros::Duration(10.0)) {
        goal_text += "x=" + std::to_string(goal_position.opencv_coords.x) +
                 ", y=" + std::to_string(goal_position.opencv_coords.y);
        } else {
        goal_text += "No click detected.";
        }

        cv::rectangle(map_image, cv::Point(box_x, box_y + 70), cv::Point(box_x + box_width, box_y + box_height + 70), cv::Scalar(255, 0, 0), 2);
        cv::putText(map_image, goal_text, cv::Point(box_x + 10, box_y + 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
        
        
        cv::resizeWindow("Map", width, height);
        cv::imshow("Map", map_image);
        cv::waitKey(1); 
    }

public:
    void run() {
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            if (isMapInitialized()) {
                drawMapWithScans();
            }
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_viewer");
    MapViewer map_viewer;
    map_viewer.run();
    return 0;
}
