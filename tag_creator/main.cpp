#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

int main(){
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(2, 2, 0.04, 0.01, dictionary);
    cv::Mat boardImage;
    board->draw( cv::Size(600, 500), boardImage, 10, 1 );
    cv::imwrite("board.png", boardImage);
    
}
