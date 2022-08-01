#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_cloud_viewer.h"
#include <pcl/visualization/pcl_visualizer.h>//for PCLVisualizer


class cloud_viewer : public QMainWindow
{
    Q_OBJECT

public:
    cloud_viewer(QWidget* parent = nullptr);

    void initViewer();
    void refreshView();
    bool endsWith(const std::string& str, const std::string suffix);
    void getAllFilesNames(std::string path, std::string suffix, std::vector<std::string>& files);
    void showCloud(std::string &path);
    void showCloud(int index);

    //setter
    void setFolderPath(std::string folder_path);
    void setIndex(int index);
    void setSuffix(std::string suffix);
    void setPointType(std::string point_type);
    

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pre_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_now_;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_next_; 
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd_pre_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_now_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd_next_;
    std::string suffix_;
    std::string pointType_;
    std::string folder_path_;
    std::vector<std::string> files_paths_;
    std::string name_cloud_now_;

    int index_;
    
private:
    Ui::cloud_viewerClass ui;


private slots:
    void slotOpen();
    void slotNext();
    void slotPre();
    void slotOpenOneCloud();
    void slotChangeSuffix();
    void slotChangePointType();

//signals:
    //void FOO();

};