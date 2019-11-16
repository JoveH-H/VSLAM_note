#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    // 读取图像和数据库
    cout<<"reading database"<<endl;
    DBoW3::Vocabulary vocab("../vocabulary.yml.gz");
    // DBoW3::Vocabulary vocab("../vocab_larger.yml.gz");
    if ( vocab.empty() )
    {
        cerr<<"Vocabulary does not exist."<<endl;
        return 1;
    }
    cout<<"reading images... "<<endl;
    vector<Mat> images; 
    for ( int i=0; i<10; i++ )
    {
        string path = "../../058/data/"+to_string(i+1)+".png";
        images.push_back( imread(path) );
    }
    
    // 在本例中，将图像与自己生成的词汇表进行比较，这可能导致过度拟合
    // 检测ORB功能
    cout<<"detecting ORB features ... "<<endl;
    Ptr< Feature2D > detector = ORB::create();
    vector<Mat> descriptors;
    for ( Mat& image:images )
    {
        vector<KeyPoint> keypoints; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
    }
    
    // 可以直接比较图像，也可以将一个图像与数据库进行比较
    // 图像 :
    cout<<"comparing images with images "<<endl;
    for ( int i=0; i<images.size(); i++ )
    {
        DBoW3::BowVector v1;
        vocab.transform( descriptors[i], v1 );
        for ( int j=i; j<images.size(); j++ )
        {
            DBoW3::BowVector v2;
            vocab.transform( descriptors[j], v2 );
            double score = vocab.score(v1, v2);
            cout<<"image "<<i<<" vs image "<<j<<" : "<<score<<endl;
        }
        cout<<endl;
    }
    
    // 数据库 :
    cout<<"comparing images with database "<<endl;
    DBoW3::Database db( vocab, false, 0);
    for ( int i=0; i<descriptors.size(); i++ )
        db.add(descriptors[i]);
    cout<<"database info: "<<db<<endl;
    for ( int i=0; i<descriptors.size(); i++ )
    {
        DBoW3::QueryResults ret;
        db.query( descriptors[i], ret, 4);      // max result = 4
        cout<<"searching for image "<<i<<" returns "<<ret<<endl<<endl;
    }
    cout<<"done."<<endl;
}
