//==== Color + Object Detection + Panelization code =====//
//										  				 //
//-------------------------------------------------------//
// Author: Akash Kothari  <akothar3@ncsu.edu>            //
//-------------------------------------------------------//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cstdint>
#include <dirent.h>
#include <fstream>
#include <string>

using namespace cv;

namespace Panelization {

// Generic class to represent colors
class Color {
// HSV values that represent colors	
	uint8_t lowH = 0;
	uint8_t highH = 0;
	uint8_t lowS = 0;
	uint8_t highS = 0;
	uint8_t lowV = 0;
	uint8_t highV = 0;
	
// Color name	
	std::string color;
public:
// Constructors	
	Color() = default;
	
	Color(uint8_t lowH, uint8_t highH, uint8_t lowS, uint8_t highS,
		  uint8_t lowV, uint8_t highV, std::string color = std::string()) {
		setColor(lowH, highH, lowS, highS, lowV, highV, color);
	}
	
// Get color info
	uint8_t getLowH() const { 
		return lowH;
	}
	
	uint8_t getHighH() const { 
		return highH;
	}
	
	uint8_t getLowS() const { 
		return lowS;
	}
	
	uint8_t getHighS() const { 
		return highS;
	}
	
	uint8_t getLowV() const { 
		return lowV;
	}
	
	uint8_t getHighV() const { 
		return highV;
	}
	
	std::string getColor() const {
		return color;
	}
	
	std::vector<uint8_t> getHSV() const {
		std::vector<uint8_t> vect;
		vect.push_back(lowH);
		vect.push_back(highH);
		vect.push_back(lowS);
		vect.push_back(highS);
		vect.push_back(lowV);
		vect.push_back(highV);
		
		return vect;
	}
	
// Set colors	
	void setColor(uint8_t lowH, uint8_t highH, uint8_t lowS, uint8_t highS,
				  uint8_t lowV, uint8_t highV, std::string color = std::string()) {
		this->lowH = lowH;
		this->highH = highH;
		this->lowS = lowS;
		this->highS = highS;
		this->lowV = lowV;
		this->highV = highV;
		this->color = color;
	}
	
// Order of parameters: LOW_H, HIGH_H, LOW_S, HIGH_S, LOW_V, HIGH_V
	void setColor(std::vector<uint8_t> &HSV_vector) {
		if(HSV_vector.size() != 6) {
			std::cout << "Invalid HSV vector\n";
			exit(-1);
		}
		
		std::vector<uint8_t> vect = HSV_vector;
		highV = vect.back();
		vect.pop_back();
		lowV = vect.back();
		vect.pop_back();
		highS = vect.back();
		vect.pop_back();
		lowS = vect.back();
		vect.pop_back();
		highH = vect.back();
		vect.pop_back();
		lowH = vect.back();
		vect.pop_back();
	}
	
	void setColor(std::string color_name) {
	// Check of the color is valid
		color = color_name;
	}
	
	void setColor(const Color &color) {
		*this = color;
	}
};

// Panel Object
class Panel {
// Panel Dimensions	
	double width = 0;
	double height = 0;
	
// Number of panels	
	uint32_t numPanels = 0;
public:
	Panel() = default;
	
	Panel(double panelWidth, double panelHeight = 0) {
		width = panelWidth;
		height = panelHeight;
	}
	
// Get panel info	
	double getWidth() const {
		return width;
	}
	
	double getHeight() const {
		return height;
	}
	
	double getNumPanels() const {
		return numPanels;
	}
	
// Set parameters	
	void incrementNumPanels(uint32_t increment = 1) {
		numPanels += increment;
	}
	
	void decrementNumPanels(uint32_t decrement = 1) {
		numPanels -= decrement;
	}
	
// Print Panel information
	void printPanelInfo() {
		std::cout << "+++++++++++++ PRINT PANEL INFO ++++++++++++++\n";
		std::cout << "Panel Width: " << width << "\n";
		std::cout << "Panel Height: " << height << "\n";
		std::cout << "Number of Panels: " << numPanels << "\n";
		std::cout << "+++++++++++++++++++++++++++++++++++++++++++++\n";
	}
};

// Wall object
class Wall: public Color {
// Wall dimensions
	double length = 0;
	double width = 0;
	double height = 0;

// Vector of vector of panels that best fit the wall
	std::vector<std::vector<Panel *> *> bestFitPanelList;
	
// We define the "head" of the panel list as the outermost panel for a wall.
// We map the head of the panels to the pointer to the panel list.
	std::map<const Panel *, std::vector<Panel *> *> panelHeadListMap;
	
// Cordinates of the wall in the print
	std::vector<Point2f> rectPoints;
public:
	Wall() = default;
	
	Wall(double length, double width, double height = 0) {
		this->length = length;
		this->width = width;
		this->height = height;
	}
	
	Wall(const Color &color) {
		this->setColor(color);
	}
	
// Get wall info	
	double getLength() const {
		return length;
	}
	
	double getWidth() const {
		return width;
	}
	
	double getHeight() const {
		return height;
	}
	
	std::vector<std::vector<Panel *> *> &getBestFitPanelList() {
		return bestFitPanelList;
	}
	
	Point2f getVertex(unsigned index) const {
		return rectPoints[index];
	}
	
// Setting all dimensions
	void setLength(double wallLength) {
		length = wallLength;
	}
	
	void setWidth(double wallWidth) {
		width = wallWidth;
	}
	
	void setHeight(double wallHeight) {
		height = wallHeight;
	}
	
	void setRectPoints(std::vector<Point2f> &wallCoordinates) {
		if(wallCoordinates.size() != 4) {
			std::cout << "Error in coordinates of wall\n";
			exit(-1);
		}
		rectPoints.clear();
		rectPoints.push_back(wallCoordinates[0]);
		rectPoints.push_back(wallCoordinates[1]);
		rectPoints.push_back(wallCoordinates[2]);
		rectPoints.push_back(wallCoordinates[3]);
	}
	
	void addBestFitPanel(Panel *panel, uint32_t numPanels = 1) {
	// This is a panel for a new list
		std::vector<Panel *> *panelList = new std::vector<Panel *>();
		panelList->push_back(panel);
		bestFitPanelList.push_back(panelList);
		panelHeadListMap[panel] = panelList;
		panel->incrementNumPanels(numPanels);
	}
	
	void addBestFitPanel(Panel *panel, std::vector<Panel*> &headPanelList, 
													uint32_t numPanels = 1) {
		if(headPanelList.empty()) {
		// This is a panel for a new list
			std::vector<Panel *> *panelList = new std::vector<Panel *>();
			panelList->push_back(panel);
			bestFitPanelList.push_back(panelList);
			
			panelHeadListMap[panel] = panelList;
			headPanelList.push_back(panel);
		} else {
		// Access the map to get the panel list corresponding to the given head panel
		// and add the new panel to the list.
			std::vector<Panel *>::iterator it = headPanelList.begin();
			while(it != headPanelList.end()) {
				(*panelHeadListMap[*it]).push_back(panel);
				it++;
			}
		}
		panel->incrementNumPanels(numPanels);
	}
	
// Verify wall info is correct
	bool wallInfoIsSane() {
		if(rectPoints.size() != 4) 
			return false;
		if(!length)
			return false;
		return true;
	}
	
// Print Wall info
	void printWallInfo() {
	// Verify wall info is sane
		if(!wallInfoIsSane()) {
			std::cout << "Wall info is not sane\n";
			exit(-1);
		}
		
		std::cout << "************ WALL INFO ******************\n";
		
	// Print wall dimensions	
		std::cout << "Wall Length: " << length << "\n";
		std::cout << "Wall Width: " << width << "\n";
		std::cout << "Wall Height: " << height << "\n";
	
	// Print wall color
		std::cout << "Wall Color: " << getColor() << "\n";
		
	// Print Best fit Panel info
		std::vector<std::vector<Panel *> *>::iterator panel_list_it = bestFitPanelList.begin();
		while(panel_list_it != bestFitPanelList.end()) {
			std::cout << "-------------------- PANEL LIST ----------------------\n";
			std::vector<Panel *>::iterator panel_it = (*panel_list_it)->begin();
			while(panel_it != (*panel_list_it)->end()) {
				(*panel_it)->printPanelInfo();
				panel_it++;
			}
			std::cout << "------------------------------------------------------\n";
			panel_list_it++;
		}
		
	// Print wall cordinates
		std::vector<Point2f>::iterator points_it = rectPoints.begin();
		while(points_it != rectPoints.end()) {
			std::cout << "(" << (*points_it).x << ", " << (*points_it).y << ")\n";
			points_it++;
		}
		std::cout << "*****************************************\n";
	}
};

// This class represents images that uses Color class. We do not want this class
// to be accessed by any class other than Processor and HSV_Tracker.
class Image {
private:
	friend class Processor;
	friend class HSV_Tracker;
	
// Image matrix	
	Mat image;
	
	Image() = default;
	
	Image(const std::string &fileName) {
		loadImage(fileName);
	}
	
	Mat &getImage() {
		return image;
	}
	
	void setImage(Mat &image) {
		this->image = image;
	}
	
	void loadImage(const std::string &fileName) {
	// Load source image and convert it to gray
		image = imread(fileName);
		if(image.empty()) {
			std::cout << "Error: Could not read image file\n";
			exit(-1);
		}
	}
	
	void convertBGR2HSV() {
		convertBGR2HSV(*this);
	}

	void convertBGR2HSV(Image &newImage) {
		Mat new_image;
		cvtColor(image, new_image, COLOR_BGR2HSV);
		newImage.setImage(new_image);
	}
	
	void displayImage(const std::string &imageName) const {
	//Pop a window to display the image
		namedWindow(imageName , WINDOW_AUTOSIZE);
		imshow(imageName, image);
	}
	
	void getMaskedImage(Image &masked_image, Color &color) {
		Scalar lowVal = Scalar(color.getLowH(), color.getLowS(), color.getLowV());
		Scalar highVal = Scalar(color.getHighH(), color.getHighS(), color.getHighV());
		Mat newImage;
		inRange(image, lowVal, highVal, newImage);
		masked_image.setImage(newImage);
	}
	
	void maskImage(Color &color) {
		getMaskedImage(*this, color);
	}

	void findContoursAndSort(std::vector<std::vector<Point>> &contours, 
											 std::vector<Vec4i> &hierarchy) {
		findContours(image, contours, hierarchy, RETR_EXTERNAL, 
										CHAIN_APPROX_SIMPLE, Point(0, 0));
		std::sort(contours.begin(), contours.end(), 
				[this](std::vector<Point> a, std::vector<Point> b) {
					return compareContourAreas(a, b); 
				});
	}
	
	void drawImageContours(Image &newImage, std::vector<std::vector<Point>> &contours, 
														std::vector<Vec4i> &hierarchy) {
	// Draw contours on an image with black background
		Mat new_image = Mat::zeros(image.size(), CV_64F);
		unsigned long i = 0;
		while(i != contours.size()) {
			drawContours(new_image, contours, i, (150, 50, 25), 2, 8, hierarchy, 0, Point());
			i++;
		}
		newImage.setImage(new_image);
	}
	
	void drawImageContours(std::vector<std::vector<Point>> &contours,
													  std::vector<Vec4i> &hierarchy) {
		 drawImageContours(*this, contours, hierarchy); 
	}
	
	void resizeImage(Image &newImage, Size outputSize = Size(), double xScale = 0, 
								double yScale = 0, int interpolation = INTER_LINEAR) {
		Mat new_image;
		resize(image, new_image, outputSize, xScale, yScale, interpolation);
		newImage.setImage(new_image);
	}
	
	void resizeImage(Size outputSize = Size(), double xScale = 0, double yScale = 0, 
			                                       int interpolation = INTER_LINEAR) {
		resizeImage(*this, outputSize, xScale, yScale, interpolation);
	}
	
	bool compareContourAreas(std::vector<Point> contour1, std::vector<Point> contour2) {
		return (fabs(contourArea(Mat(contour1))) < fabs(contourArea(Mat(contour2))));
	}
};

// Most of the functions of Processor are private so outside functions or 
// classes cannot directly use it. If any function that involves interaction
// with the front-end could be added under public acess-modifier.
class Processor {
// Wall list
	std::vector<Wall *> wallList;
	
// List of panels
	std::vector<Panel> panelList;
	
// List of valid colors
	std::vector<Color> validColorList;

// Map to map names of colors and HSV values to see if colors are valid
	std::map<const std::string, bool> colorNameMap;
	std::map<const std::vector<uint8_t>, bool> HSVMap;
	
// Map color names to HSV values
	std::map<const std::string, std::vector<uint8_t>> colorNameHSVMap;
public:
	Processor(const std::string &imageName, const std::string &outputFileName = std::string(), 
			  double scale = 0.1, double noiseThreshold = 10) {
	// Get the image
		Image image(imageName);
		image.convertBGR2HSV();
		
	// List of avaliable panels 
		addPanel(2, 1);
		addPanel(1.5, 1);
		addPanel(0.5, 1);
		
	// WARNING: A few colors are hard coded. Make sure these colors work.
	// Use Color Pick or Gpick for tuning colors.
		addColor(120, 120, 40, 255, 0, 255, "Blue");   //Blue (Works)
		addColor(18, 56, 0, 255, 0, 255, "Yellow");  // Yellow
		addColor(58, 66, 0, 255, 0, 255, "Green"); // Green
		
	// Start detecting walls 	
		std::vector<Color>::iterator it = validColorList.begin();
		while(it != validColorList.end()) {
		// Get masked images
			Image masked_image;
			image.getMaskedImage(masked_image, *it);
			masked_image.displayImage("Masked Image");
			
		// Find contours of the given color	
			std::vector<std::vector<Point>> contours;
			std::vector<Vec4i> hierarchy;
			masked_image.findContoursAndSort(contours, hierarchy);
		
		// Box the contours found and get wall cordinates
			boxContours(contours, *it);
			waitKey(10000);
			it++;
		}
		
	// Filter out the noise	
		filterNoise(scale, noiseThreshold);
		
	// Compute panels needed for given wall layout
		printAvailablePanels();
		computeNumPanelsBestFit();
		
	// Write data to output file
		writeToFile(outputFileName, imageName);
		
	// Print Walls and panels
		printWallList();
	}
	
// Get data collected by Processor
	const std::vector<Wall *> &getWallList() const {
		return wallList;
	}
	
// Print panel list
	void printAvailablePanels() {
		std::cout << "||||||||||||||| PRINTING AVAILABLE PANELS |||||||||||||||||\n";
		std::vector<Panel>::iterator it = panelList.begin();
		while(it != panelList.end()) {
			std::cout << "+++++++++++++ PRINTING PANEL INFO ++++++++++++++\n";
			std::cout << "Panel Width: " << (*it).getWidth() << "\n";
			std::cout << "Panel Height: " << (*it).getHeight() << "\n";
			std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++\n";
			it++;
		}
		std::cout << "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n";
	}
	
// Print wall list
	void printWallList() {
		std::cout << "===================== PRINTING WALL LIST ===================\n";
		std::vector<Wall *>::iterator it = wallList.begin();
		while(it != wallList.end()) {
			(*it)->printWallInfo();
			it++;
		}
		std::cout << "============================================================\n";
	}
	
// Print valid color list
	void printValidColorList() {
		std::cout << "-------------------- PRINTING COLOR LIST --------------------\n";
		std::vector<Color>::iterator it = validColorList.begin();
		while(it != validColorList.end()) {
			std::cout << (*it).getColor() << "\n";
			it++;
		}
		std::cout << "-------------------------------------------------------------\n";
	}
private:
// API to add panels to available panel list
	void addPanel(double width, double height) {
		Panel panel(width, height);
		panelList.push_back(panel);
	}
	
// API to add colors to valid colors list
	void addColor(uint8_t lowH, uint8_t highH, uint8_t lowS, uint8_t highS,
				  uint8_t lowV, uint8_t highV, const std::string &colorName) {
	// Build Color info	
		Color color(lowH, highH, lowS, highS, lowV, highV, colorName);
		
	// Add the color to the list	
		validColorList.push_back(color);
		
	// Add the color name to the valid color map	
		colorNameMap[colorName] = true;
		
	// Add HSV values to map	
		std::vector<uint8_t> vect;
		vect.push_back(lowH);
		vect.push_back(highH);
		vect.push_back(lowS);
		vect.push_back(highS);
		vect.push_back(lowV);
		vect.push_back(highV);
		HSVMap[vect] = true;
		
	// Map color name with HSV values
		colorNameHSVMap[colorName] = vect;
	}

	void boxContours(std::vector<std::vector<Point>> &contours, const Color &color) {
	// Store the contour rectangles in an array	
		std::vector<RotatedRect> minRect(contours.size());	
		unsigned long i = 0;
		while(i != contours.size()) {
			minRect[i] = minAreaRect(Mat(contours[i]));
			
		// Allocate wall
			Wall *wall = allocateWall(color);
			
		// Get the rectangle points and add it to the allocated wall
			Point2f rect_points[4];
			minRect[i].points(rect_points);
			std::vector<Point2f> vect;
			vect.push_back(rect_points[0]);
			vect.push_back(rect_points[1]);
			vect.push_back(rect_points[2]);
			vect.push_back(rect_points[3]);
			wall->setRectPoints(vect);
			
		// Add the wall to wall list
			wallList.push_back(wall);
			
			i++;
		}
	}
	
	Wall *allocateWall(const Color &color) {
		Wall *wall = new Wall(color);
		if(!wall) {
			std::cout << "Error in allocating wall.\n";
			exit(-1);
		}
		return wall;
	}
	
	Wall *allocateWall(double length = 0, double width = 0, double height = 0) {
		Wall *wall = new Wall(length, width, height);
		if(!wall) {
			std::cout << "Error in allocating wall.\n";
			exit(-1);
		}
		return wall;
	}
	
	Panel *allocatePanel(double width = 0, double height = 0) {
		Panel *panel = new Panel(width, height);
		if(!panel) {
			std::cout << "Error in allocating panel.\n";
			exit(-1);
		}
		return panel;
	}

	void computeNumPanelsBestFit() {
	// Iterate over the wall list and compute the number of panels needed
		std::vector<Wall *>::iterator wall = wallList.begin();
		while(wall != wallList.end()) {
		// Check if any panel type fits perfectly with the half wall of given length. 
		// If not, we keep track of error and panel width the lowest leftover wall space
		// to get the "best" panel. Put half panels in vector too.
			double wallLength = ((*wall)->getLength()) / 2;
			std::cout << "****************************************\n";
			std::cout << "WALL LENGTH: " << 2 * wallLength << "\n";
			double extraSpace = wallLength;
			double smallerExtraSpace;
			int zeroSpaceFitFound = -1;
			std::vector<Panel> panelVect = panelList;
			std::vector<Panel>::iterator panel = panelList.begin();
			while(panel != panelList.end()) {
				Panel temp_panel(((*panel).getWidth()) / 2);
				panelVect.push_back(temp_panel);
				panel++;
			}
			panel = panelVect.begin();
			std::vector<Panel *> headPanelList;
			std::vector<Panel *> perfectFitPanelList;
			std::vector<std::vector<double>> panelsAdded;
			while(panel != panelVect.end()) {
				double panelWidth = panel->getWidth();
				if(wallLength >= panelWidth) {
					if(zeroSpaceFitFound == -1)
						zeroSpaceFitFound = 0;
					uint32_t numPanels = wallLength / panelWidth;
					double spaceLeft = wallLength - (numPanels * panelWidth);
					std::cout << "PANEL WIDTH: " << panelWidth << "\n";
					std::cout << "SPACE LEFT: " << spaceLeft << "\n";
					if(!spaceLeft) {
					// Perfect match found! Allocate panel. But before that, check if
					// its a half panel.
						zeroSpaceFitFound = 1;
						std::vector<Panel>::iterator temp_panel = panelList.begin();
						while(temp_panel != panelList.end()) {
							if(panelWidth == (*temp_panel).getWidth()) {
								Panel *panel = allocatePanel(panelWidth);
								numPanels = 2 * (wallLength / panelWidth);
								(*wall)->addBestFitPanel(panel, numPanels);
								std::cout << "NUM PANELS: " << numPanels << "\n";
								headPanelList.push_back(panel);
								goto next_panel;
							}
							temp_panel++;
						}
						
					// Check if the full panel corresponding to the full panel has already
					// been added to the wall. If yes, we can move on.
						panelWidth *= 2;
						std::vector<Panel *>::iterator it = headPanelList.begin();
						while(it != headPanelList.end()) {
							if((*it)->getWidth() == panelWidth)
								goto next_panel;
							it++;
						}
						Panel *panel = allocatePanel(panelWidth);
						numPanels = ((uint32_t)(2 * (wallLength / panelWidth)) - 1) + 1;
						(*wall)->addBestFitPanel(panel, numPanels);
						std::cout << "NUM PANELS: " << numPanels << "\n";	
						goto next_panel;
					}
					//std::cout << "BUGGER\n";
					std::vector<Panel>::iterator temp_panel = panelList.begin();
					while(temp_panel != panelList.end()) {
						if(panelWidth == (*temp_panel).getWidth()) {
						// Get the lowest space left over
							std::cout << "EXTRA SPACE: " << extraSpace << "\n";
							if(spaceLeft == extraSpace) {
								//std::cout << "BUGGER\n";
								std::vector<double> temp_vect;
								temp_vect.push_back(spaceLeft);
								temp_vect.push_back(panelWidth);
								panelsAdded.push_back(temp_vect);
								extraSpace = spaceLeft;
								std::cout << "EXTRA\n";
								std::cout << "PANELS ADDED VECT SIZE: " << panelsAdded.size() << "\n";
							} else {
								if(spaceLeft < extraSpace) {
									std::cout << "SPACE LEFT LESS THAN EXTRA SPACE\n";
									std::vector<std::vector<double>>::iterator it = panelsAdded.begin();
									if(it != panelsAdded.end()) {
										while(it != panelsAdded.end()) {
											if((*it)[0] == extraSpace) {
												(*it)[0] = spaceLeft;
												(*it)[1] = panelWidth;
											}
											it++;
										}
									} else {
										std::vector<double> temp_vect;
										temp_vect.push_back(spaceLeft);
										temp_vect.push_back(panelWidth);
										panelsAdded.push_back(temp_vect);
									}
									extraSpace = spaceLeft;
									std::cout << "ppEXTRA\n";
									std::cout << "PANELS ADDED VECT SIZE: " << panelsAdded.size() << "\n";
								}
							}
							break;
						}
						temp_panel++;
					}
				}
			next_panel:
				panel++;
			}
			
		// If the best fit with zero space left is not found deal with the panels that work	
			if(!zeroSpaceFitFound && !panelsAdded.empty()) {
				std::vector<std::vector<double>>::iterator it = panelsAdded.begin();
				while(it != panelsAdded.end()) {
					Panel *panel = allocatePanel((*it)[1]);
					uint32_t numPanels = 2 * (uint32_t)(wallLength / (*it)[1]);
					(*wall)->addBestFitPanel(panel, numPanels);
					std::cout << "zPANEL WIDTH: " <<  (*it)[1] << "\n";
					std::cout << "zNUM PANELS: " << numPanels << "\n";
					//extraSpace = wallLength - ((uint32_t)(wallLength / (*it)[1]) * (*it)[1]);
					
				// Since this is a head panel, we add this to the head panel list
					headPanelList.push_back(panel);
					it++;
				}
			} else {  
				goto next_wall;
			}
			
		// Try to fit panels in the left over space. Half panels would do as well.
			//extraSpace = wallLength - ((uint32_t)(wallLength / optPanelWidth) * optPanelWidth);
			std::cout << "EXTRA SPACE LEFT: " << 2 * extraSpace << "\n";
			zeroSpaceFitFound = -1;
			panelsAdded.clear();
			smallerExtraSpace = extraSpace;
			panel = panelVect.begin();
			while(panel != panelVect.end()) {
				double panelWidth = panel->getWidth();
				if(extraSpace >= panelWidth) {
					if(zeroSpaceFitFound == -1)
						zeroSpaceFitFound = 0;
					uint32_t numPanels = extraSpace / panelWidth;
					double spaceLeft = extraSpace - (numPanels * panelWidth);
					std::cout << "--PANEL WIDTH: " << panelWidth << "\n";
					std::cout << "--SPACE LEFT: " << spaceLeft << "\n";
					if(!spaceLeft) {
					// Perfect match found! Allocate panel. But before that, check if
					// its a half panel.
						zeroSpaceFitFound = 1;
						std::vector<Panel>::iterator temp_panel = panelList.begin();
						while(temp_panel != panelList.end()) {
							if(panelWidth == (*temp_panel).getWidth()) {
								Panel *panel = allocatePanel(panelWidth);
								numPanels = 2 * (extraSpace / panelWidth);
								(*wall)->addBestFitPanel(panel, headPanelList, numPanels);
								std::cout << "--NUM PANELS: " << numPanels << "\n";
								perfectFitPanelList.push_back(panel);
								goto next_panel2;
							}
							temp_panel++;
						}
						panelWidth *= 2;
						std::vector<Panel *>::iterator it = perfectFitPanelList.begin();
						while(it != perfectFitPanelList.end()) {
							if((*it)->getWidth() == panelWidth)
								goto next_panel2;
							it++;
						}
						Panel *panel = allocatePanel(panelWidth);
						numPanels = ((uint32_t)(2 * (wallLength / panelWidth)) - 1) + 1;
						(*wall)->addBestFitPanel(panel, headPanelList, numPanels);
						std::cout << "--NUM PANELS: " << numPanels << "\n";
						goto next_panel2;
					}
					
					if(spaceLeft == smallerExtraSpace) {
						std::vector<double> temp_vect;
						temp_vect.push_back(spaceLeft);
						temp_vect.push_back(panelWidth);
						panelsAdded.push_back(temp_vect);
						smallerExtraSpace = spaceLeft;
						std::cout << "--EXTRA\n";
					} else {
						if(spaceLeft < smallerExtraSpace) {
							std::vector<std::vector<double>>::iterator it = panelsAdded.begin();
							if(it != panelsAdded.end()) {
								while(it != panelsAdded.end()) {
									if((*it)[0] == smallerExtraSpace) {
										(*it)[0] = spaceLeft;
										(*it)[1] = panelWidth;
									}
									it++;
								}
							} else {
								std::vector<double> temp_vect;
								temp_vect.push_back(spaceLeft);
								temp_vect.push_back(panelWidth);
								panelsAdded.push_back(temp_vect);
							}
							smallerExtraSpace = spaceLeft;
							std::cout << "--ppEXTRA\n";
						}
					}
					
				}
			next_panel2:
				panel++;
			}
			
		// If the best fit with zero space left is not found deal with the panels that work	
			if(!zeroSpaceFitFound && !panelsAdded.empty()) {
				std::vector<std::vector<double>>::iterator it = panelsAdded.begin();
				while(it != panelsAdded.end()) {
					double panelWidth = (*it)[1];
					uint32_t numPanels;
					
				// Check if it is half panel
					std::vector<Panel>::iterator panel_it = panelList.begin();
					while(panel_it != panelList.end()) {
						if(panel_it->getWidth() == panelWidth) {
							numPanels = 2 * (uint32_t)(extraSpace / panelWidth);
							goto allocate_panel;
						}
						panel_it++;
					}
					panelWidth *= 2;
					numPanels = ((uint32_t)(2 * (extraSpace / panelWidth)) - 1) + 1;
					
				allocate_panel:	
					Panel *panel = allocatePanel(panelWidth);
					(*wall)->addBestFitPanel(panel, headPanelList, numPanels);
					std::cout << "--zPANEL WIDTH: " <<  (*it)[1] << "\n";
					std::cout << "--zNUM PANELS: " << numPanels << "\n";
					it++;
				}
			}
	
		next_wall:
			std::cout << "***********************************************\n";
			wall++;
		}
	}

	void filterNoise(double scale, double noiseThreshold) {
	// Filter noise. Avoid anything that does not have a proper, significant size
	// Traverse the object list and filter the bugger noise out. For rectangles,
	// there are only three unique distances. We pick two shortest lengths, ignoring
	// the longest one which is most likely to be the diagonal.
		std::vector<Wall *> tempList = wallList;
		wallList.clear();
		std::vector<Wall *>::iterator wall_it = tempList.begin();
		while(wall_it != tempList.end()) {
		// Walls are treated as rectangles. Get two rectangle sides	and check if the ratio of
		// sides is more than the threshold. If it is, put the wall in the wall list and save 
		// the scaled lengths of the walls.
			double side1 = euclideanDistance((*wall_it)->getVertex(0), (*wall_it)->getVertex(1));
			double side2 = euclideanDistance((*wall_it)->getVertex(1), (*wall_it)->getVertex(2));
			if(side1 && side2) {
				if(side1 > side2) {
					if(side1 / side2 >= noiseThreshold) {
						if(side2 <= 1000 && side2 >= 5) {
							wallList.push_back(*wall_it);
							(*wall_it)->setLength(side1 * scale);
						}
						std::cout << " WALL SIDE WIDTH: " << side1 << "\n";	
					}
				} else {
					if(side2 / side1 >= noiseThreshold) {
						if(side1 <= 1000 && side1 >= 5) {
							wallList.push_back(*wall_it);
							(*wall_it)->setLength(side2 * scale);
						}
						std::cout << " WALL SIDE WIDTH: " << side1 << "\n";	
					}
				}
			}
			wall_it++;
		}
	}
	
// Write all the collected data to a file
	void writeToFile(const std::string &outputFileName, const std::string &imageName) {
	// If the output file name has not be given, we use the image name to make output file name	
		std::string output;
		if(outputFileName.empty()) {
			output = imageName;
			std::string::iterator it = output.begin();
			std::string::iterator dot_it = output.begin();
			while(it != output.end()) {
				if(*it == '.')
					dot_it = it;
				it++;
			} 
			std::cout << "OUPUT FILE NAME: " << output << "\n";
		// Erase image extension
			it = dot_it + 1;
			while(it != output.end()) {
				output.erase(it);
				it = dot_it + 1;
			}
			std::cout << "OUPUT FILE NAME: " << output << "\n";
		// Append new extension
			output.append("csv");
		} else {
			output = outputFileName;
		}
		std::cout << "OUPUT FILE NAME: " << output << "\n";
	// Write to output file	
		std::ofstream outputFile;
		outputFile.open(output);
		outputFile << "Wall Length (ft),Wall Width (ft),Wall Height (ft),"
						"Panel Width (ft),Number of Panels" << std::endl;
		std::vector<Wall *>::iterator wall = wallList.begin();
		while(wall != wallList.end()) {
		// Write wall dimensions
			std::ostringstream strs;
			std::string str;
			str.append("\"");
			strs << (*wall)->getLength();
			str.append(strs.str());
			str.append("\"");
			outputFile << str << ","; 
			std::ostringstream strs2;
			str.clear();
			str.append("\"");
			strs2 << (*wall)->getWidth();
			str.append(strs2.str());
			str.append("\"");
			outputFile << str << ",";
			std::ostringstream strs3;
			str.clear();
			str.append("\"");
			strs3 << (*wall)->getHeight();
			str.append(strs3.str());
			str.append("\"");
			outputFile << str << ",";
			
			std::cout << "WALL DIMENSIONS ADDED\n";
		// List the panels
			std::vector<std::vector<Panel *> *>::iterator panel_list = 
										((*wall)->getBestFitPanelList()).begin();
			while(panel_list != ((*wall)->getBestFitPanelList()).end()) {
				std::string str;
				str.append("\"");
				std::vector<Panel *>::iterator panel = (*panel_list)->begin();
				while(panel != (*panel_list)->end()) {
					std::ostringstream strs;
					strs << (*panel)->getWidth();
					str.append(strs.str());
					str.append(", ");
					panel++;
				}
				std::string::reverse_iterator rit = str.rbegin() + 1;
				std::cout << "LAST CHAR: " << *rit << "\n";
				*rit = '\"';
				outputFile << str;
				std::cout << "STRING: " << str << "\n";
				std::cout << "PANEL WIDTH ADDED\n";
				outputFile << ",";
				std::string str2;
				str2.append("\"");
				panel = (*panel_list)->begin();
				while(panel != (*panel_list)->end()) {
					std::ostringstream strs;
					strs << (*panel)->getNumPanels();
					str2.append(strs.str());
					str2.append(", ");
					panel++;
				}
				rit = str2.rbegin() + 1;
				std::cout << "LAST CHAR: " << *rit << "\n";
				*rit = '\"';
				outputFile << str2;
				std::cout << "STRING: " << str2 << "\n";
				std::cout << "PANEL NUM ADDED\n";
				outputFile << std::endl;
				panel_list++;
				if(panel_list != ((*wall)->getBestFitPanelList()).end()) {
					outputFile << ",,,";
				}
			}
			outputFile << std::endl;
			wall++;
		}
		
	// Done wrtiting to the file. Save the god damn thing.	
		outputFile.close();
	}
	
// Calculate the euclidean distance between the points
	double euclideanDistance(Point2f point1, Point2f point2) const {
		return norm(point1 - point2);
	}

// API to check the validity of color	
	bool isValidColor(uint8_t lowH, uint8_t highH, uint8_t lowS, uint8_t highS,
					  uint8_t lowV, uint8_t highV) {
		std::vector<uint8_t> vect;
		vect.push_back(lowH);
		vect.push_back(highH);
		vect.push_back(lowS);
		vect.push_back(highS);
		vect.push_back(lowV);
		vect.push_back(highV);
		return HSVMap[vect];
	}
	
	bool isValidColor(std::vector<uint8_t> hsvVect) {
		if(hsvVect.size() != 6) {
			std::cout << "Invalid color\n";
			exit(-1);
		}
		return HSVMap[hsvVect];
	}
	
	bool isValidColor(std::string colorName) {
		std::vector<Color>::iterator it = validColorList.begin();
		while(it != validColorList.end()) {
			if(colorName == (*it).getColor())
				return true;
			it++;
		}
		return false;
	}
};

// Class for making the HSV tracker to pop up and enable a user to adjust HSV values
class HSV_Tracker {
public:	
	HSV_Tracker(const std::string &imageName, const std::string &winName = "HSV_Tracker") {
	// Create a window with track bars on it
		createTrackbarWindow(winName);

	// Analyse image
		Image image(imageName);
		image.convertBGR2HSV();
		while(1) {
		// Get the positions of the trackbars
			int lowH = getTrackbarPos("lowH", winName);
			int highH = getTrackbarPos("highH", winName);
			int lowS = getTrackbarPos("lowS", winName);
			int highS = getTrackbarPos("highS", winName);
			int lowV = getTrackbarPos("lowV", winName);
			int highV = getTrackbarPos("highV", winName);
			Color color(lowH, highH, lowS, highS, lowV, highV);
			Image newImage;
			image.getMaskedImage(newImage, color);
			newImage.resizeImage(Size(), 0.75, 0.75);
			newImage.displayImage(winName);
			waitKey(1000);  //large wait time to avoid freezing
		}
	}
	
private:
	void createTrackbarWindow(const std::string &windowName) const {
	// Create a window
		namedWindow(windowName);
		
	// Create trackbars	
		int min_slide_val = 0;
		int max_slide_val = 180;
		createTrackbar("lowH", windowName, &min_slide_val, 180);
		createTrackbar("highH", windowName, &max_slide_val, 180);
		max_slide_val = 255;
		createTrackbar("lowS", windowName, &min_slide_val, 255);
		createTrackbar("highS", windowName, &max_slide_val, 255);
		createTrackbar("lowV", windowName, &min_slide_val, 255);
		createTrackbar("highV", windowName, &max_slide_val, 255);
	}
};

} // namespace Panelization

int main(int argc, char** argv )
{
	Panelization::Processor p(argv[1]);
	//Panelization::HSV_Tracker(argv[1], "Trackwin");
	return 0;
}
