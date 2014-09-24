FILE(REMOVE_RECURSE
  "/home/youbot/uva_at_work_catkin/src/speech_recognizer/lcmtypes/c"
  "/home/youbot/uva_at_work_catkin/src/speech_recognizer/lcmtypes/cpp"
  "/home/youbot/uva_at_work_catkin/src/speech_recognizer/lcmtypes/java"
  "/home/youbot/uva_at_work_catkin/src/speech_recognizer/lcmtypes/python"
  "CMakeFiles/lcmgen_python"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/lcmgen_python.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
