/**********************************************************
 Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 This software is released under BSD license as expressed below
-------------------------------------------------------------------
Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:

   This product includes software developed by the Ava group of the University of Cordoba.

4. Neither the name of the University nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************/


#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/functional.hpp>
#include "raspicam_cv.h"
using namespace cv;
using namespace std;

bool doTestSpeedOnly=false;
constexpr short multicast_port = 30001;
constexpr short server_port = 30002;

class receiver
{
public:
  receiver(boost::asio::io_context& io_context,
      const boost::asio::ip::address& listen_address,
      const boost::asio::ip::address& multicast_address,
      raspicam::RaspiCam_Cv& Cam)
    : socket_(io_context) , io_context_(io_context) , Camera(Cam)
  {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        listen_address, multicast_port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    // Join the multicast group.
    socket_.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));

    do_receive();
  }

private:
  void do_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_), sender_endpoint_,
        [this](boost::system::error_code ec, std::size_t length)
        {
          if (!ec)
          {
            // std::cout.write(data_.data(), length);
            // std::cout << " " << sender_endpoint_.address().to_string() ;
            // std::cout << std::endl;

            double t0 = double (cv::getTickCount());
            Camera.grab();
            Camera.retrieve ( image );
            image = imread("test_contour4.jpg",cv::IMREAD_GRAYSCALE);
            
            std::string request_id(data_.data() , length);
            processImage(image , [this](std::vector<std::vector<cv::Point> > &contours){
                send_to_server(contours , sender_endpoint_.address())
            });
            double t1 = double (cv::getTickCount());
            double secondsElapse = (t1 - t0) / double ( cv::getTickFrequency() );
            float fps = ( float ) ( ( float ) ( 1 ) / secondsElapse );
            std::cout << "FPS: " <<  fps << std::endl;

            do_receive();
          }
        });
    }

    void send_to_server(std::vector<std::vector<cv::Point> > &contours , boost::asio::ip::address address)
    {
        std::cout << contours.size() << " " << address.to_string() << std::endl;
        boost::asio::ip::tcp::socket sock(io_context_);
        boost::asio::ip::tcp::endpoint sender_endpoint(address, server_port);
        boost::system::error_code error;
        sock.connect(sender_endpoint, error);

        if(error){
            std::cerr << boost::system::system_error(error).what() << std::endl;
            return;
        }

        std::cout << "Connected!!" << std::endl;
    }

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::array<char, 20> data_;
  boost::asio::io_context& io_context_;
  raspicam::RaspiCam_Cv &Camera;
  cv::Mat image;
};

//parse command line
//returns the index of a command line param in argv. If not found, return -1
int findParam ( string param,int argc,char **argv ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    return idx;

}
//parse command line
//returns the value of a command line param. If not found, defvalue is returned
float getParamVal ( string param,int argc,char **argv,float defvalue=-1 ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    if ( idx==-1 ) return defvalue;
    else return atof ( argv[  idx+1] );
}

void processCommandLine ( int argc,char **argv,raspicam::RaspiCam_Cv &Camera ) {
    Camera.set ( cv::CAP_PROP_FRAME_WIDTH,  getParamVal ( "-w",argc,argv,1280 ) );
    Camera.set ( cv::CAP_PROP_FRAME_HEIGHT, getParamVal ( "-h",argc,argv,720 ) );
    Camera.set ( cv::CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
    Camera.set ( cv::CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
    Camera.set ( cv::CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
    Camera.set ( cv::CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
    Camera.set ( cv::CAP_PROP_FPS, getParamVal ( "-fps",argc,argv, 100 ) );
    // if ( findParam ( "-gr",argc,argv ) !=-1 )
    Camera.set ( cv::CAP_PROP_FORMAT, CV_8UC1 );
    if ( findParam ( "-test_speed",argc,argv ) !=-1 )
        doTestSpeedOnly=true;
    if ( findParam ( "-ss",argc,argv ) !=-1 )
        Camera.set ( cv::CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv )  );


//     Camera.setSharpness ( getParamVal ( "-sh",argc,argv,0 ) );
//     if ( findParam ( "-vs",argc,argv ) !=-1 )
//         Camera.setVideoStabilization ( true );
//     Camera.setExposureCompensation ( getParamVal ( "-ev",argc,argv ,0 ) );


}

void showUsage() {
    cout<<"Usage: "<<endl;
    cout<<"[-gr set gray color capture]\n";
    cout<<"[-test_speed use for test speed and no images will be saved]\n";
    cout<<"[-w width] [-h height] \n[-br brightness_val(0,100)]\n";
    cout<<"[-co contrast_val (0 to 100)]\n[-sa saturation_val (0 to 100)]";
    cout<<"[-g gain_val  (0 to 100)]\n";
    cout<<"[-ss shutter_speed (0 to 100) 0 auto]\n";
    cout<<"[-fps frame_rate (0 to 120) 0 auto]\n";
    cout<<"[-nframes val: number of frames captured (100 default). 0 == Infinite lopp]\n";

    cout<<endl;
}

void findImageContours(cv::Mat &image , boost::function<void(std::vector<std::vector<cv::Point> >&)> callback){
    cv::Mat thresh;
    cv::threshold(image, thresh, 200, 255, cv::THRESH_BINARY );//+ cv::THRESH_OTSU);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
    callback(contours);
}

void processImage(cv::Mat &image , boost::function<void(std::vector<std::vector<cv::Point> >&)> callback){
    int W = image.size().width;
    int halfW = W/2;

    int H = image.size().height;
    int halfH = H/2;
    
    cv::Rect rec1(0,0, halfW, halfH);
    cv::Rect rec2(halfW,0, halfW, halfH);
    cv::Rect rec3(0,halfH, halfW, halfH);
    cv::Rect rec4(halfW,halfH, halfW, halfH);
    
    Mat cropped1 = image(rec1);
    Mat cropped2 = image(rec2);
    Mat cropped3 = image(rec3);
    Mat cropped4 = image(rec4);

    boost::thread process1(findImageContours , cropped1 , callback);
    boost::thread process2(findImageContours , cropped2 , callback);
    boost::thread process3(findImageContours , cropped3 , callback);
    boost::thread process4(findImageContours , cropped4 , callback);

    // process1.join();
    // process2.join();
    // process3.join();
    // process4.join();
}

int main ( int argc,char **argv ) {
    if ( argc==1 ) {
        cerr<<"Usage (-help for help)"<<endl;
    }
    if ( findParam ( "-help",argc,argv ) !=-1 ) {
        showUsage();
        return -1;
    }

    raspicam::RaspiCam_Cv Camera;
    processCommandLine ( argc,argv,Camera );
    bool serve = false;
    if ( findParam ( "-serve",argc,argv ) !=-1 ) {
        serve = true;
    }

    cout<<"Connecting to camera"<<endl;
    if ( !Camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;

    try
    {
        if(serve){
            boost::asio::io_context io_context;
            receiver r(io_context,
            boost::asio::ip::make_address("0.0.0.0"),
            boost::asio::ip::make_address("239.255.0.1"),
            Camera);
            io_context.run();
        }
    }catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    int nCount=getParamVal ( "-nframes",argc,argv, 100 );
    cout<<"Capturing"<<endl;

    cv::Mat image;
    double time_=cv::getTickCount();
    double thresholdFPS = 0;
    double contourFPS = 0;
    for ( int i=0; i<nCount || nCount==0; i++ ) {
        Camera.grab();
        Camera.retrieve ( image );
        // / cv::Mat img = imread("test_contour4.jpg",cv::IMREAD_GRAYSCALE);
        // image = imread("test_contour4.jpg",cv::IMREAD_GRAYSCALE);
        

        double t0 = 0;
        double t1 = 0;

        // cv::Mat thresh;
        //// cv::adaptiveThreshold(image,thresh,255,cv::ADAPTIVE_THRESH_MEAN_C , cv::THRESH_BINARY, 11, 0 );
        //// adaptiveThreshold(image,thresh,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,11,0);

        // t0 = double (cv::getTickCount());
        // cv::threshold(image, thresh, 200, 255, cv::THRESH_BINARY );//+ cv::THRESH_OTSU);
        // std::vector<std::vector<cv::Point> > contours;
        // cv::findContours( thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
        // t1 = double (cv::getTickCount());

        contourFPS += double (t1-t0);

        t0 = double (cv::getTickCount());
        processImage(image , [](std::vector<std::vector<cv::Point> > &contours){

        });
        t1 = double (cv::getTickCount());
        thresholdFPS += double (t1-t0);

        // cv::Scalar color(255, 0, 0);
        // for (size_t idx = 0; idx < contours.size(); idx++) {
        //     // cv::drawContours(image, contours, idx, color);
        // }

        if ( !doTestSpeedOnly ) {
            if ( i%5==0 ) 	  cout<<"\r capturing ..."<<i<<"/"<<nCount<<std::flush;
            if ( i%30==0 && i!=0 ){
                cv::imwrite ("image"+std::to_string(i)+".jpg",image );
            }
        }
    }

    if ( !doTestSpeedOnly )  cout<<endl<<"Images saved in imagexx.jpg"<<endl;
    double secondsElapsed= double ( cv::getTickCount()-time_ ) /double ( cv::getTickFrequency() ); //time in second
    double thresholdSecondsElapse = thresholdFPS / double ( cv::getTickFrequency() );
    double contourSecondsElapse = contourFPS / double ( cv::getTickFrequency() );

    cout<< secondsElapsed<<" seconds for "<< nCount<<"  frames : FPS = "<< ( float ) ( ( float ) ( nCount ) /secondsElapsed ) << "  MultiThread FPS: " << ( float ) ( ( float ) ( nCount ) /thresholdSecondsElapse )  << "  SingleThread FPS: "  << ( float ) ( ( float ) ( nCount ) /contourSecondsElapse ) <<endl;
    Camera.release();

}
