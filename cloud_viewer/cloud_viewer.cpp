#include "cloud_viewer.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <vtkGenericOpenGLRenderWindow.h>
//for getAllFilesNames()
#include <pcl/point_cloud.h>
#include <io.h>
#include <vector>
#include <string>
#include <iostream>
#include <qfiledialog.h>//for QFileDialog

cloud_viewer::cloud_viewer(QWidget* parent)
	: QMainWindow(parent)
{
	//参数初始化
	suffix_ = ".ply";
	pointType_ = "PointXYZRGB";
	index_ = 0;
	folder_path_ = "C:/Users/1/Desktop";

	//设置ui
	ui.setupUi(this);
	//初始化显示窗口
	initViewer();

	//信号-槽
	connect(ui.button_next, SIGNAL(clicked()), this, SLOT(slotNext()));
	connect(ui.button_pre, SIGNAL(clicked()), this, SLOT(slotPre()));
	connect(ui.Button_open, SIGNAL(clicked()), this, SLOT(slotOpen()));
	connect(ui.button_open_item, SIGNAL(clicked()), this, SLOT(slotOpenOneCloud()));
	connect(ui.comboBox_suffix, SIGNAL(currentIndexChanged()), this, SLOT(slotChangeSuffix()));
	connect(ui.comboBox_pointType, SIGNAL(currentIndexChanged()), this, SLOT(slotChangePointType()));

	//渲染显示组件
	refreshView();
}

void cloud_viewer::initViewer()
{
	//读取一个简单点云
	//cloud_now_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::io::loadPLYFile<pcl::PointXYZRGB>("PeterRabbit004.ply", *cloud_now_);
	//设置QVTK窗口
	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	//设置PCLVisualizer
	viewer_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
	viewer_->initCameraParameters();
	viewer_->resetCamera();
	viewer_->setBackgroundColor(0, 0, 0);
	//viewer_->addPointCloud(cloud_now_, "cloud");
	//viewer_->addText("PeterRabbit004.ply",10,10,"url");
	viewer_->addText("url",10,10,"url");
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//连接QVTK和PCLVisualizer
	ui.VTKWidget_1->setRenderWindow(viewer_->getRenderWindow());
	viewer_->setupInteractor(ui.VTKWidget_1->interactor(), ui.VTKWidget_1->renderWindow());

	refreshView();
}

void cloud_viewer::refreshView()
{
	ui.VTKWidget_1->renderWindow()->Render();
}

bool cloud_viewer::endsWith(const std::string& str, const std::string suffix) {
	if (suffix.length() > str.length()) { return false; }

	return (str.rfind(suffix) == (str.length() - suffix.length()));
}

void cloud_viewer::getAllFilesNames(std::string path, std::string suffix, std::vector<std::string>& files)
{
	//文件句柄
	intptr_t   hFile = 0;
	//文件信息 
	struct _finddata_t fileinfo;
	std::string p;
	//if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是目录,检查文件名后缀，符合要求的存入Vector
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				// 因为在系统中打开一个文件夹前两个默认是“.”和".."（就像在压缩文件中看到的一样），如果不加这一句会导致无限迭代。打印一下fileinfo.name即可明白。
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					//getAllFilesNames(p.assign(path).append("\\").append(fileinfo.name), suffix, files);
					getAllFilesNames(p.assign(path).append("/").append(fileinfo.name), suffix, files);
			}
			else
			{
				if (endsWith(fileinfo.name, suffix))
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name));
					files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void cloud_viewer::showCloud(std::string& path)
{
	suffix_ = ui.comboBox_suffix->currentText().toStdString();
	pointType_ = ui.comboBox_pointType->currentText().toStdString();

	if (suffix_==".ply")
	{
		if (pointType_=="PointXYZRGB")
		{
			if (cloud_now_==NULL)
			{
				cloud_now_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			}
			if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(path, *cloud_now_) == 0)
			{
				viewer_->removeAllPointClouds();
				viewer_->initCameraParameters();
				viewer_->resetCamera();
				viewer_->addPointCloud(cloud_now_, "cloud_now_");
				viewer_->updateText(path, 10, 20, "url");
			}
		}
		else
		{
			if (cloud_xyz_now_ == NULL)
			{
				cloud_xyz_now_.reset(new pcl::PointCloud<pcl::PointXYZ>);
			}
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud_xyz_now_) == 0)
			{
				viewer_->removeAllPointClouds();
				viewer_->initCameraParameters();
				viewer_->resetCamera();
				viewer_->addPointCloud(cloud_xyz_now_, "cloud_xyz_now_");
				viewer_->updateText(path, 10, 10, "url");
			}
		}

	}
	else if (suffix_==".pcd")
	{
		if (pointType_ == "PointXYZRGB")
		{
			if (cloud_now_ == NULL)
			{
				cloud_now_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
			}
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud_now_) == 0)
			{
				viewer_->removeAllPointClouds();
				viewer_->initCameraParameters();
				viewer_->resetCamera();
				viewer_->addPointCloud(cloud_now_, "cloud_now_");
				viewer_->updateText(path, 10, 10, "url");
			}
		}
		else
		{
			if (cloud_xyz_now_ == NULL)
			{
				cloud_xyz_now_.reset(new pcl::PointCloud<pcl::PointXYZ>);
			}
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud_xyz_now_) == 0)
			{
				viewer_->removeAllPointClouds();
				viewer_->initCameraParameters();
				viewer_->resetCamera();
				viewer_->addPointCloud(cloud_xyz_now_, "cloud_xyz_now_");
				viewer_->updateText(path, 10, 10, "url");
			}
		}
	}

	refreshView();
}

void cloud_viewer::showCloud(int index)
{
	showCloud(files_paths_.at(index));
}

void cloud_viewer::setFolderPath(std::string folder_path)
{
	this->folder_path_ = folder_path;
}

void cloud_viewer::setIndex(int index)
{
	this->index_ = index;
}

void cloud_viewer::setSuffix(std::string suffix)
{
	this->suffix_ = suffix;
}

void cloud_viewer::setPointType(std::string point_type)
{
	this->pointType_ = point_type;
}

void cloud_viewer::slotOpen()
{
	QString file_folder = QFileDialog::getExistingDirectory(this, tr("open folder"), "/home", QFileDialog::ShowDirsOnly);
	setFolderPath(file_folder.toStdString());
	suffix_ = ui.comboBox_suffix->currentText().toStdString();
	pointType_ = ui.comboBox_pointType->currentText().toStdString();
	files_paths_.clear();
	getAllFilesNames(folder_path_, suffix_, files_paths_);
	setIndex(0);
	showCloud(index_);
}

void cloud_viewer::slotNext()
{
	if ((index_ + 1) < files_paths_.size())
	{
		index_++;
		showCloud(files_paths_.at(index_));
	}
	else
	{
		//std::cout << "error" << std::endl;
	}
}

void cloud_viewer::slotPre()
{
	if ((index_ - 1) >= 0)
	{
		index_--;
		showCloud(files_paths_.at(index_));
	}
	else
	{
		//std::cout << "error" << std::endl;
	}
}

void cloud_viewer::slotOpenOneCloud()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));
	std::string filepath = filename.toStdString();
	showCloud(filepath);
}

void cloud_viewer::slotChangeSuffix()
{
	this->suffix_ = ui.comboBox_suffix->currentText().toStdString();
}

void cloud_viewer::slotChangePointType()
{
	this->pointType_ = ui.comboBox_pointType->currentText().toStdString();
}
