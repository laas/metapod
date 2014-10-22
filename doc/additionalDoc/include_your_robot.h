/** \page include_your_robot_model How to include a metapod model of your robot in your code

   In the remainder of this section, it is assumed that you are using CMake 
   as a compilation environment.

   For a continuous integration of your model in your code, a good practice is
   to generate automatically the C++ code for your robot based on the URDF model. <br>
   To do that you should add the following line to the CMakeLists.txt of your project: <br>
   
   \code
   include(path_to_your_metapod_install/share/cmake/metapod.cmake)
   \endcode

   Then at the CMakeLists.txt where you need to use the model,
   you should add:
   \code
   set(model_name "the_name_of_your_robot")
   add_sampleurdfmodel(${model_name} ${urdf_path} ${data_path} ${gen_dir}$ )
   \endcode

   where 
   <ul>
   <li>\a the_name_of_your_robot is the prefix of your robot model file, </li>
   <li> \a urdf_path is the directory where the \a ${name}.config file and the file \a metapod_license_file.txt 
   are located. </li>
   <li> \a gen_dir is the location where the headers and the sources files
   are generated. </li>
   </ul>
   If you do not specify ${urdf_path}, ${data_path} and ${gen_dir} then they are set to the following default
   values:
   <ul>
     <li> \a urdf_path: ${PROJECT_SOURCE_DIR}/data/${name}.urdf</li>
     <li> \a data_path: ${PROJECT_SOURCE_DIR}/data/</li>
     <li> \a gen_dir: ${CMAKE_CURRENT_DIR}/include/metapod/models/</li>
   </ul>

   To force cmake to create the C++ headers and code related to your robot model, you should add or modify 
   the TARGET_LINK_LIBRARIES of your target:
   \code
   TARGET_LINK_LIBRARIES(your_target metapod_${model_name} ${the_other_libraries})
   \endcode
   
   To see how to modify the code depending on your target, please look at the tutorials page.
 */
