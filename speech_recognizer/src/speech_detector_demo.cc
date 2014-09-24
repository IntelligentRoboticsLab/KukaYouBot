#include <getopt.h>
#include <iostream>
#include <QtGui/QApplication>

#include "ros/ros.h"

#include "speech_detector.h"

using namespace std;
using namespace h2sl;

static void
print_usage (const char *name)
{
    fprintf (stderr, "usage: %s [options]\n"
             "\n"
             " -l, --language   <LANGUAGE>   Spoken language\n"
             " -s, --samplerate <SAMPLERATE> Sample rate in Hz\n"
             " -v, --verbose               Verbose outout\n"
             " -h, --help                    Print this help and exit\n"
             "\n",
             name);
}

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Speech_Detector class demo program" << endl;

  ros::init(argc, argv, "speech_recognizer");
  ros::NodeHandle node;
  
  QApplication app( argc, argv );


  const char *optstring = "hvs:l:";
  int c;
  struct option long_opts[] = 
  {
      { "help", no_argument, 0, 'h'},
      { "verbose", no_argument, 0, 'v'},
      { "language", required_argument, 0, 'l'},
      { "samplerate", required_argument, 0, 's'},
      { 0, 0, 0, 0 }
  };

  string language = "en-US";
  int samplerate = 8000;
  bool verbose = false;
  while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
      switch (c) {
      case 'l':
          language = strdup (optarg);
          break;
      case 's':
          samplerate = strtol (optarg, NULL, 10);
          break;
      case 'v':
          verbose = true;
          break;
      case 'h':
      default:
          print_usage (argv[0]);
          return 1;
      }
  }
    

  Speech_Detector speech_detector( language, samplerate, verbose );
  speech_detector.show();

  return app.exec();
}
