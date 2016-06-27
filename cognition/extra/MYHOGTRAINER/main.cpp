#include <stdio.h>
#include <dirent.h>

#ifdef __MINGW32__
#include <sys/stat.h>
#endif

#include <ios>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include "svmlight/svmlight.h"

using namespace std;
using namespace cv;


static string posSamplesDir = "possleep/";
static string negSamplesDir = "negsleep/";
static string negFirstSamplesDir = "negsleepfirst/";
static string featuresFile = "genfiles/features.dat";
static string svmModelFile = "genfiles/svmlightmodel.dat";
static string descriptorVectorFile = "genfiles/descriptorvector.dat";
static string cvHOGFile = "genfiles/cvHOGClassifier.yaml";
static string treshFile = "genfiles/treshHold.txt";

static string posTestsDir = "postestsleep/";
static string negTestsDir = "negtestsleep/";

static const Size trainingPadding = Size(0, 0);
static const Size winStride = Size(8, 8);

static const Size hogWinSize = Size(192, 64);
static const Size minPyramid = Size(192, 64);
static const Size slidSize = Size(192, 64);
static const double pyramidScale = 1.60;
static const int slidingStepx = 80;
static const int slidingStepy = 32;
static const int neghnmrepeat = 2;
static const int poshnmrepeat = 0;

static const int maxDetectionWindow = 400;
static const double nmsTresh = 0.65;

static string toLowerCase(const string &in) {
    string t;
    for (string::const_iterator i = in.begin(); i != in.end(); ++i) {
        t += tolower(*i);
    }
    return t;
}

static void getFilesInDirectory(const string &dirName, vector<string> &fileNames,
                                const vector<string> &validExtensions) {
    printf("Opening directory %s\n", dirName.c_str());
#ifdef __MINGW32__
    struct stat s;
#endif
    struct dirent *ep;
    size_t extensionLocation;
    DIR *dp = opendir(dirName.c_str());
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
#ifdef __MINGW32__
            stat(ep->d_name, &s);
            if (s.st_mode & S_IFDIR) {
                continue;
            }
#else
            if (ep->d_type & DT_DIR) {
                continue;
            }
#endif
            extensionLocation = string(ep->d_name).find_last_of(".");
            string tempExt = toLowerCase(string(ep->d_name).substr(extensionLocation + 1));
            if (find(validExtensions.begin(), validExtensions.end(), tempExt) != validExtensions.end()) {
                //printf("Found matching data file '%s'\n", ep->d_name);
                fileNames.push_back((string) dirName + ep->d_name);
            } else {
                printf("Found file does not match required file type, skipping: '%s'\n", ep->d_name);
            }
        }
        (void) closedir(dp);
    } else {
        printf("Error opening directory '%s'!\n", dirName.c_str());
    }
    return;
}

static void cropImage(Mat &imageData, Mat &cropdImage, Rect &box) {
    if (box.x >= 0 && box.width > 0 && (box.x + box.width) <= imageData.cols && box.y >= 0 && box.height >= 0 &&
        (box.y + box.height) <= imageData.rows) {
        cropdImage = imageData(box);
    }
}

static void cropImage(Mat &imageData, Mat &cropdImage, int x, int y, int width, int height) {
    if (x >= 0 && y >= 0) {
        Rect box(x, y, width, height);
        cropImage(imageData, cropdImage, box);
    }
}

static void resizeScl(Mat &imageData, Mat &resizedImage, double scale) {
    resize(imageData, resizedImage, Size(imageData.cols * scale, imageData.rows * scale), 0, 0, INTER_LINEAR);
}

static void resizeToDetectSize(Mat &imageData, Mat &resizedImage, int maxD) {
    //double scale = min(1.00, maxD / (double) (max(imageData.rows, imageData.cols)));
    double scale = maxD / (double) (max(imageData.rows, imageData.cols));
    resizeScl(imageData, resizedImage, scale);
}

static void calculateHogFeatures(Mat &imageData, vector<float> &featureVector, HOGDescriptor &hog) {
    if ((imageData.cols != hog.winSize.width) || (imageData.rows != hog.winSize.height)) {
        printf("[%d][%d]invalid image size\n", imageData.cols, imageData.rows);
        return;
    }

    vector<Point> locations;
    hog.compute(imageData, featureVector, winStride, trainingPadding, locations);
    imageData.release();
}

static void saveDescriptorVectorToFile(vector<float> &descriptorVector, vector<unsigned int> &_vectorIndices,
                                       string fileName) {
    printf("Saving descriptor vector to file '%s'\n", fileName.c_str());
    string separator = " ";
    fstream File;
    File.open(fileName.c_str(), ios::out);
    if (File.good() && File.is_open()) {
        for (int feature = 0; feature < descriptorVector.size(); ++feature) {
            File << descriptorVector.at(feature) << separator;
        }
        File << endl;
        File.flush();
        File.close();
    }
}

vector<Rect> non_max_suppression(vector<Rect> rects, int overlapThresh) {
    vector<vector<int> > boxes;
    for (int i = 0; i < rects.size(); i++) {
        vector<int> box;
        box.push_back(rects[i].x);
        box.push_back(rects[i].y);
        box.push_back(rects[i].x + rects[i].width);
        box.push_back(rects[i].y + rects[i].height);
        boxes.push_back(box);
    }
    vector<Rect> pick;
    if (boxes.size() == 0)return pick;
    while (boxes.size() > 0) {
        int maxy2idx = 0;
        vector<int> maxy2box = boxes[maxy2idx];

        for (int i = 1; i < boxes.size(); i++) {
            vector<int> box = boxes[i];
            if (box[3] > maxy2box[3]) {
                maxy2idx = i;
                maxy2box = box;
            }
        }

        boxes.erase(boxes.begin() + maxy2idx);

        pick.push_back(Rect(maxy2box[0], maxy2box[1], maxy2box[2] - maxy2box[0], maxy2box[3] - maxy2box[1]));

        for (int i = 0; i < boxes.size(); i++) {
            vector<int> box = boxes[i];

            int xx1 = max(maxy2box[0], box[0]);
            int yy1 = max(maxy2box[1], box[1]);
            int xx2 = min(maxy2box[2], box[2]);
            int yy2 = min(maxy2box[3], box[3]);

            int w = max(0, xx2 - xx1 + 1);
            int h = max(0, yy2 - yy1 + 1);

            double overlap = ((double) (w * h)) / ((box[0] - box[2]) * (box[1] - box[3]));

            if (overlap > overlapThresh)boxes.erase(boxes.begin() + i);
        }
    }
    return pick;
}

static void testDetector(vector<string> &posFileNames, vector<string> &negFileNames, HOGDescriptor &hog,
                         double treshold) {
    unsigned int truePositives = 0;
    unsigned int trueNegatives = 0;
    unsigned int falsePositives = 0;
    unsigned int falseNegatives = 0;
    Size padding(Size(8, 8));
    vector<Rect> foundDetection;
    // Walk over positive training samples, generate images and detect
    for (vector<string>::const_iterator posTrainingIterator = posFileNames.begin();
         posTrainingIterator != posFileNames.end(); ++posTrainingIterator) {
        Mat imageData = imread(*posTrainingIterator, IMREAD_GRAYSCALE);
        resizeToDetectSize(imageData, imageData, maxDetectionWindow);
        hog.detectMultiScale(imageData, foundDetection, treshold, winStride, padding);
        foundDetection = non_max_suppression(foundDetection, nmsTresh);
        if (foundDetection.size() > 0) {
            ++truePositives;
            falseNegatives += foundDetection.size() - 1;
        } else {
            ++falseNegatives;
        }
    }
    // Walk over negative training samples, generate images and detect
    for (vector<string>::const_iterator negTrainingIterator = negFileNames.begin();
         negTrainingIterator != negFileNames.end(); ++negTrainingIterator) {
        Mat imageData = imread(*negTrainingIterator, IMREAD_GRAYSCALE);
        resizeToDetectSize(imageData, imageData, maxDetectionWindow);
        hog.detectMultiScale(imageData, foundDetection, treshold, winStride, padding);
        foundDetection = non_max_suppression(foundDetection, nmsTresh);
        if (foundDetection.size() > 0) {
            falsePositives += foundDetection.size();
        } else {
            ++trueNegatives;
        }
    }

    printf("Results:\n\tTrue Positives: %u\n\tTrue Negatives: %u\n\tFalse Positives: %u\n\tFalse Negatives: %u\n",
           truePositives, trueNegatives, falsePositives, falseNegatives);
}

int main(int argc, char **argv) {

    HOGDescriptor hog;
    hog.winSize = hogWinSize;

    static vector<string> positiveTrainingImages;
    static vector<string> negativeTrainingImages;
    static vector<string> positiveTestImages;
    static vector<string> negativeTestImages;
    static vector<string> negativeFirstTestImages;

    static vector<string> validExtensions;
    validExtensions.push_back("jpg");
    validExtensions.push_back("png");
    validExtensions.push_back("ppm");

    getFilesInDirectory(posSamplesDir, positiveTrainingImages, validExtensions);
    getFilesInDirectory(negSamplesDir, negativeTrainingImages, validExtensions);
    getFilesInDirectory(posTestsDir, positiveTestImages, validExtensions);
    getFilesInDirectory(negTestsDir, negativeTestImages, validExtensions);
    getFilesInDirectory(negFirstSamplesDir, negativeFirstTestImages, validExtensions);

    if ((positiveTrainingImages.size() + negativeTrainingImages.size()) == 0) {
        printf("No training sample files found, nothing to do!\n");
        return EXIT_SUCCESS;
    }

    //calculate training images features and write them to feature file
    fstream File;
    File.open(featuresFile.c_str(), ios::out);
    if (File.good() && File.is_open()) {
        //calculate positive images features
        for (unsigned long currentFile = 0; currentFile < positiveTrainingImages.size(); ++currentFile) {            
            Mat imageData = imread(positiveTrainingImages.at(currentFile), IMREAD_GRAYSCALE);

            if (!imageData.empty()) {
                //cropImage(imageData, imageData, (imageData.cols - hog.winSize.width) / 2,
                          //(imageData.rows - hog.winSize.height) / 2, hog.winSize.width, hog.winSize.height);
		resize(imageData, imageData, Size(hog.winSize.width, hog.winSize.height), 0, 0, INTER_LINEAR);
		Mat flp;
		flip(imageData, flp, 1);
		vector<float> featureVectorimg;
		vector<float> featureVectorflp;

	        calculateHogFeatures(imageData, featureVectorimg, hog);

	        if (!featureVectorimg.empty()) {
	            File << "+1";
	            for (unsigned int feature = 0; feature < featureVectorimg.size(); ++feature) {
	                File << " " << (feature + 1) << ":" << featureVectorimg.at(feature);
	            }
	            File << endl;
	        }
		
		calculateHogFeatures(flp, featureVectorflp, hog);

	        if (!featureVectorflp.empty()) {
	            File << "+1";
	            for (unsigned int feature = 0; feature < featureVectorflp.size(); ++feature) {
	                File << " " << (feature + 1) << ":" << featureVectorflp.at(feature);
	            }
	            File << endl;
	        }
            }
        }
        //calculate negative images features
        for (unsigned long currentFile = 0; currentFile < negativeFirstTestImages.size(); ++currentFile) {
            vector<float> featureVector;
            Mat imageData = imread(negativeFirstTestImages.at(currentFile), IMREAD_GRAYSCALE);

            if (!imageData.empty()) {
                resize(imageData, imageData, Size(hog.winSize.width, hog.winSize.height), 0, 0, INTER_LINEAR);
                calculateHogFeatures(imageData, featureVector, hog);

                if (!featureVector.empty()) {
                    File << "-1";
                    for (unsigned int feature = 0; feature < featureVector.size(); ++feature) {
                        File << " " << (feature + 1) << ":" << featureVector.at(feature);
                    }
                    File << endl;
                }
            }
        }

        File.flush();
        File.close();
    } else {
        printf("Error opening file '%s'!\n", featuresFile.c_str());
        return EXIT_FAILURE;
    }

    vector<float> descriptorVector;
    vector<unsigned int> descriptorVectorIndices;
    double hitThreshold;

    for (int hnmrep = 0; hnmrep < neghnmrepeat; hnmrep++) {
        //using svmlight to train descriptor model
        SVMlight *svm1 = SVMlight::getInstance();
        printf("Calling %s\n", svm1->getSVMName());
        svm1->read_problem(const_cast<char *> (featuresFile.c_str()));
        svm1->train();
        printf("First training done!\n");
        //svm->saveModelToFile(svmModelFile);

        printf("Generating representative single HOG feature vector using svmlight!\n");
        descriptorVector.clear();
        descriptorVectorIndices.clear();
        svm1->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
        //saveDescriptorVectorToFile(descriptorVector, descriptorVectorIndices, descriptorVectorFile);

        hitThreshold = svm1->getThreshold();

        hog.setSVMDetector(descriptorVector);
        //hog.save(cvHOGFile);

        printf("Applying hard-negative mining\n");
        File.open(featuresFile.c_str(), ios::out | ios::app);
        if (File.good() && File.is_open()) {
            for (unsigned long currentFile = 0; currentFile < negativeTrainingImages.size(); ++currentFile) {
                Mat imageData = imread(negativeTrainingImages.at(currentFile), IMREAD_GRAYSCALE);

                if (!imageData.empty()) {
                    resizeToDetectSize(imageData, imageData, maxDetectionWindow);
                    while (imageData.cols >= minPyramid.width && imageData.rows >= minPyramid.height) {
                        for (int y = 0; y <= (imageData.rows - slidSize.height); y += ((slidingStepy / (hnmrep+1))+(5*hnmrep))) {
                            for (int x = 0; x <= (imageData.cols - slidSize.width); x += ((slidingStepx / (hnmrep+1))+(5*hnmrep))) {
                                Mat slide;
                                cropImage(imageData, slide, x, y, slidSize.width, slidSize.height);

                                if (!slide.empty()) {
                                    resize(slide, slide, Size(hog.winSize.width, hog.winSize.height), 0, 0,
                                           INTER_LINEAR);

                                    vector<Point> foundDetection;
                                    hog.detect(slide, foundDetection, hitThreshold, winStride, trainingPadding);
                                    if (foundDetection.size() > 0) {
                                        vector<float> featureVector;
                                        calculateHogFeatures(slide, featureVector, hog);
                                        if (!featureVector.empty()) {
                                            File << "-1";
                                            for (unsigned int feature = 0; feature < featureVector.size(); ++feature) {
                                                File << " " << (feature + 1) << ":" << featureVector.at(feature);
                                            }
                                            File << endl;
                                        }
                                    }
                                } else {
                                    printf("ERROR: slide window is empty\n");
                                }
                            }
                        }

                        resizeScl(imageData, imageData, 1 / pyramidScale);
                    }
                } else {
                    printf("ERROR: couldn't load image at: %s", negativeTestImages.at(currentFile).c_str());
                }
            }
            File.flush();
            File.close();
        } else {
            printf("Error opening file '%s' to append hard-negative features!\n", featuresFile.c_str());
            return EXIT_FAILURE;
        }
    }

    for (int hnmrep = 0; hnmrep < poshnmrepeat; hnmrep++) {
        //using svmlight to train descriptor model
        SVMlight *svm1 = SVMlight::getInstance();
        printf("Calling %s\n", svm1->getSVMName());
        svm1->read_problem(const_cast<char *> (featuresFile.c_str()));
        svm1->train();
        printf("First training done!\n");
        //svm->saveModelToFile(svmModelFile);

        printf("Generating representative single HOG feature vector using svmlight!\n");
        descriptorVector.clear();
        descriptorVectorIndices.clear();
        svm1->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
        //saveDescriptorVectorToFile(descriptorVector, descriptorVectorIndices, descriptorVectorFile);

        hitThreshold = svm1->getThreshold();

        hog.setSVMDetector(descriptorVector);
        //hog.save(cvHOGFile);

        printf("Applying hard-negative mining\n");
        File.open(featuresFile.c_str(), ios::out | ios::app);
        if (File.good() && File.is_open()) {
            for (unsigned long currentFile = 0; currentFile < positiveTrainingImages.size(); ++currentFile) {
                Mat imageData = imread(positiveTrainingImages.at(currentFile), IMREAD_GRAYSCALE);
                if (!imageData.empty()) {
                    //cropImage(imageData, imageData, (imageData.cols - hog.winSize.width) / 2,
                              //(imageData.rows - hog.winSize.height) / 2, hog.winSize.width, hog.winSize.height);
		    resize(imageData, imageData, Size(hog.winSize.width, hog.winSize.height), 0, 0, INTER_LINEAR);
                    vector<Point> foundDetection;
                    hog.detect(imageData, foundDetection, hitThreshold, winStride, trainingPadding);
                    if (foundDetection.size() == 0) {
                        vector<float> featureVector;
                        calculateHogFeatures(imageData, featureVector, hog);
                        if (!featureVector.empty()) {
                            File << "+1";
                            for (unsigned int feature = 0; feature < featureVector.size(); ++feature) {
                                File << " " << (feature + 1) << ":" << featureVector.at(feature);
                            }
                            File << endl;
                        }
                    }
                } else {
                    printf("ERROR: couldn't load image at: %s", negativeTestImages.at(currentFile).c_str());
                }
            }
            File.flush();
            File.close();
        } else {
            printf("Error opening file '%s' to append hard-negative features!\n", featuresFile.c_str());
            return EXIT_FAILURE;
        }
    }

    //using svmlight to train descriptor model
    SVMlight *svm2 = SVMlight::getInstance();
    printf("Calling %s\n", svm2->getSVMName());
    svm2->read_problem(const_cast<char *> (featuresFile.c_str()));
    svm2->train();
    printf("Training done, saving model file!\n");
    svm2->saveModelToFile(svmModelFile);

    printf("Generating representative single HOG feature vector using svmlight!\n");
    descriptorVector.clear();
    descriptorVectorIndices.clear();
    svm2->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
    saveDescriptorVectorToFile(descriptorVector, descriptorVectorIndices, descriptorVectorFile);

    hitThreshold = svm2->getThreshold();
    printf("treshold: %f\n", hitThreshold);
    fstream tFile;
    tFile.open(treshFile.c_str(), ios::out);
    if (tFile.good() && tFile.is_open()) {
        //tFile.precision(numeric_limits<double>::max_digits10);
        std::stringstream strT;
        strT << fixed << std::setprecision(15) << hitThreshold;
        tFile << strT.str();
        tFile << endl;
        tFile.flush();
        tFile.close();
    }
    hog.setSVMDetector(descriptorVector);
    hog.save(cvHOGFile);
    //testing trained detector
    printf("testing trained detector using training set as test set\n");
    testDetector(positiveTrainingImages, negativeTrainingImages, hog, hitThreshold);

    printf("testing trained detector using test set\n");
    testDetector(positiveTestImages, negativeTestImages, hog, hitThreshold);

    return EXIT_SUCCESS;
}
