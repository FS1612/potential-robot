#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <vector>

class ObstacleDrawer {
public:
    ObstacleDrawer() {
        scan_sub = nh.subscribe("/base_scan", 1, &ObstacleDrawer::scanCallback, this);
        odom_sub = nh.subscribe("/odom", 1, &ObstacleDrawer::odomCallback, this);

        // Inizializza la mappa OpenCV con un buffer di 1000x1000 (adattabile) e colore nero
        map_image = cv::Mat::zeros(1000, 1000, CV_8UC3); // 1000x1000, nero

        // Calcola il centro dell'immagine
        center_x = map_image.cols / 2;
        center_y = map_image.rows / 2;

        // Scala per conversione delle coordinate
        map_scale = 20.0; // Adatta questa scala a seconda delle esigenze
        
        // Publisher per la mappa degli ostacoli
        obstacle_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);

        // Inizializza la mappa di occupazione
        initializeOccupancyGrid();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Publisher obstacle_map_pub; // Publisher per pubblicare la mappa degli ostacoli
    cv::Mat map_image;
    double robot_x = 0;
    double robot_y = 0;
    double robot_theta = 0;
    int center_x;
    int center_y;
    double map_scale;

    nav_msgs::OccupancyGrid obstacle_map; // Mappa degli ostacoli
    
    // Inizializza la mappa di occupazione
    void initializeOccupancyGrid() {
        int width = map_image.cols;
        int height = map_image.rows;

        // Imposta le informazioni di base sulla mappa di occupazione
        obstacle_map.info.width = width;
        obstacle_map.info.height = height;
        obstacle_map.info.resolution = 1.0 / map_scale;
        obstacle_map.info.origin.position.x = -center_x / map_scale;
        obstacle_map.info.origin.position.y = -center_y / map_scale;

        // Inizializza la mappa con valori sconosciuti
        obstacle_map.data.resize(width * height, -1);
    }

    // Callback per odometria, aggiornamento della posizione del robot
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
        double qw = msg->pose.pose.orientation.w;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;

        // Calcolo dell'angolo di orientamento del robot
        robot_theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    // Callback per LaserScan, disegna i punti rappresentanti gli ostacoli
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg) {
        for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
            float range = scan_msg->ranges[i];
            if (std::isinf(range) || std::isnan(range) || range > scan_msg->range_max) {
                continue; // Ignora i valori non validi o troppo distanti
            }

            // Calcola l'angolo per ciascun punto in base all'angolo del robot e all'angolo del LaserScan
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment + robot_theta;

            // Converti range e angolo in coordinate (x, y)
            float x_stage = range * std::cos(angle) + robot_x;
            float y_stage = range * std::sin(angle) + robot_y;

            // Converti le coordinate Stage in coordinate OpenCV
            int x_opencv = static_cast<int>(x_stage * map_scale) + center_x;
            int y_opencv = -static_cast<int>(y_stage * map_scale) + center_y;

            // Assicurarsi che il punto sia all'interno dell'immagine e della mappa
            if (x_opencv >= 0 && x_opencv < map_image.cols &&
                y_opencv >= 0 && y_opencv < map_image.rows) {
                cv::circle(map_image, cv::Point(x_opencv, y_opencv), 3, cv::Scalar(255, 0, 0), -1); // Blu per gli ostacoli

                // Aggiorna la mappa di occupazione con valore "occupato"
                int map_index = y_opencv * map_image.cols + x_opencv;
                obstacle_map.data[map_index] = 100; // 100 significa "occupato"
            }
        }

        // Pubblica la mappa di occupazione aggiornata
        obstacle_map_pub.publish(obstacle_map);

        // Mostra l'immagine OpenCV aggiornata
        //cv::imshow("Obstacle Map", map_image);
        //cv::waitKey(1); // Breve attesa per processare la GUI
    }

public:
    void run() {
        ros::Rate loop_rate(10); // Frequenza del loop
        while (ros::ok()) {
            ros::spinOnce(); // Processa i messaggi ROS
            loop_rate.sleep(); // Pausa nel loop
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_drawer");

    ObstacleDrawer drawer;
    drawer.run(); // Avvia il ciclo principale

    return 0;
}