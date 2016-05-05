#include <stdio.h>
#include <dirent.h>
#include <ios>
#include <fstream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#define SVMLIGHT 1
#define LIBSVM 2

//#define TRAINHOG_USEDSVM SVMLIGHT
#define TRAINHOG_USEDSVM SVMLIGHT
#include "svmlight/svmlight.h"
#define TRAINHOG_SVM_TO_TRAIN SVMlight


using namespace std;
using namespace cv;


static string posSamplesDir = "pos/";
static string negSamplesDir = "neg/";
static string featuresFile = "genfiles/features.dat";
static string svmModelFile = "genfiles/svmlightmodel.dat";
static string descriptorVectorFile = "genfiles/descriptorvector.dat";
static string cvHOGFile = "genfiles/cvHOGClassifier.yaml";
static string thresholdFile = "genfiles/Threshold.txt";

static const Size trainingPadding = Size(0, 0);
static const Size winStride = Size(8, 8);

static string toLowerCase(const string& in) {
    string t;
    for (string::const_iterator i = in.begin(); i != in.end(); ++i) {
        t += tolower(*i);
    }
    return t;
}

static void storeCursor(void) {
    printf("\033[s");
}

static void resetCursor(void) {
    printf("\033[u");
}

static void saveDescriptorVectorToFile(vector<float>& descriptorVector, vector<unsigned int>& _vectorIndices, string fileName) {
    printf("Saving descriptor vector to file '%s'\n", fileName.c_str());
    string separator = " "; // Use blank as default separator between single features
    fstream File;
    float percent;
    File.open(fileName.c_str(), ios::out);
    if (File.good() && File.is_open()) {
        printf("Saving %lu descriptor vector features:\t", descriptorVector.size());
        storeCursor();
        for (int feature = 0; feature < descriptorVector.size(); ++feature) {
            if ((feature % 10 == 0) || (feature == (descriptorVector.size()-1)) ) {
                percent = ((1 + feature) * 100 / descriptorVector.size());
                printf("%4u (%3.0f%%)", feature, percent);
                fflush(stdout);
                resetCursor();
            }
            File << descriptorVector.at(feature) << separator;
        }
        printf("\n");
        File << endl;
        File.flush();
        File.close();
    }
}

static void getFilesInDirectory(const string& dirName, vector<string>& fileNames, const vector<string>& validExtensions) {
    printf("Opening directory %s\n", dirName.c_str());

    struct dirent* ep;
    size_t extensionLocation;
    DIR* dp = opendir(dirName.c_str());
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            if (ep->d_type & DT_DIR) {
                continue;
            }
            extensionLocation = string(ep->d_name).find_last_of(".");
            string tempExt = toLowerCase(string(ep->d_name).substr(extensionLocation + 1));
            if (find(validExtensions.begin(), validExtensions.end(), tempExt) != validExtensions.end()) {
                printf("Found matching data file '%s'\n", ep->d_name);
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

static void calculateFeaturesFromInput(const string& imageFilename, vector<float>& featureVector, HOGDescriptor& hog) {
    Mat imageData = imread(imageFilename, IMREAD_GRAYSCALE);
    if (imageData.empty()) {
        featureVector.clear();
        printf("Error: HOG image '%s' is empty, features calculation skipped!\n", imageFilename.c_str());
        return;
    }
    if (imageData.cols != hog.winSize.width || imageData.rows != hog.winSize.height) {
        Rect myROI((imageData.cols - hog.winSize.width)/2, (imageData.rows - hog.winSize.height)/2, 64, 128);
        imageData = imageData(myROI);
    }
    vector<Point> locations;
    hog.compute(imageData, featureVector, winStride, trainingPadding, locations);
    imageData.release();
}

int main(int argc, char** argv) {

    HOGDescriptor hog;
    hog.winSize = Size(64, 128);
    static vector<string> positiveTrainingImages;
    static vector<string> negativeTrainingImages;
    static vector<string> validExtensions;
    validExtensions.push_back("jpg");
    validExtensions.push_back("png");
    validExtensions.push_back("ppm");

    getFilesInDirectory(posSamplesDir, positiveTrainingImages, validExtensions);
    getFilesInDirectory(negSamplesDir, negativeTrainingImages, validExtensions);
    unsigned long overallSamples = positiveTrainingImages.size() + negativeTrainingImages.size();

    if (overallSamples == 0) {
        printf("No training sample files found, nothing to do!\n");
        return EXIT_SUCCESS;
    }

    setlocale(LC_ALL, "C");
    setlocale(LC_NUMERIC,"C");
    setlocale(LC_ALL, "POSIX");

    printf("Reading files, generating HOG features and save them to file '%s':\n", featuresFile.c_str());
    float percent;

    fstream File;
    File.open(featuresFile.c_str(), ios::out);
    if (File.good() && File.is_open()) {
        #if TRAINHOG_USEDSVM == SVMLIGHT
            File << "# Use this file to train, e.g. SVMlight by issuing $ svm_learn -i 1 -a weights.txt " << featuresFile.c_str() << endl;
        #endif
        for (unsigned long currentFile = 0; currentFile < overallSamples; ++currentFile) {
            storeCursor();
            vector<float> featureVector;
            const string currentImageFile = (currentFile < positiveTrainingImages.size() ? positiveTrainingImages.at(currentFile) : negativeTrainingImages.at(currentFile - positiveTrainingImages.size()));
            if ( (currentFile+1) % 10 == 0 || (currentFile+1) == overallSamples ) {
                percent = ((currentFile+1) * 100 / overallSamples);
                printf("%5lu (%3.0f%%):\tFile '%s'", (currentFile+1), percent, currentImageFile.c_str());
                fflush(stdout);
                resetCursor();
            }
            calculateFeaturesFromInput(currentImageFile, featureVector, hog);
            if (!featureVector.empty()) {
                File << ((currentFile < positiveTrainingImages.size()) ? "+1" : "-1");
                for (unsigned int feature = 0; feature < featureVector.size(); ++feature) {
                    File << " " << (feature + 1) << ":" << featureVector.at(feature);
                }
                File << endl;
            }
        }
        printf("\n");
        File.flush();
        File.close();
    } else {
        printf("Error opening file '%s'!\n", featuresFile.c_str());
        return EXIT_FAILURE;
    }
    printf("Calling %s\n", TRAINHOG_SVM_TO_TRAIN::getInstance()->getSVMName());
    TRAINHOG_SVM_TO_TRAIN::getInstance()->read_problem(const_cast<char*> (featuresFile.c_str()));
    TRAINHOG_SVM_TO_TRAIN::getInstance()->train(); // Call the core libsvm training procedure
    printf("Training done, saving model file!\n");
    TRAINHOG_SVM_TO_TRAIN::getInstance()->saveModelToFile(svmModelFile);
    printf("Generating representative single HOG feature vector using svmlight!\n");
    vector<float> descriptorVector;
    vector<unsigned int> descriptorVectorIndices;
    TRAINHOG_SVM_TO_TRAIN::getInstance()->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
    saveDescriptorVectorToFile(descriptorVector, descriptorVectorIndices, descriptorVectorFile);
    const double hitThreshold = TRAINHOG_SVM_TO_TRAIN::getInstance()->getThreshold();
    printf("\n----------------------threshold  %f\n", hitThreshold);
    hog.setSVMDetector(descriptorVector);
    hog.save(cvHOGFile);

    fstream File1;    
    File1.open(thresholdFile.c_str(), ios::out);
    if (File1.good() && File1.is_open()) {
        File1 << hitThreshold;
        File1 << endl;
        File1.flush();
        File1.close();
    }

    return EXIT_SUCCESS;
}
