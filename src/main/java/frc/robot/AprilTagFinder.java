/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
package frc.robot;


 import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;
 import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.List;
 
 public class AprilTagFinder{
     private final AprilTagDetector m_detector = new AprilTagDetector();
    ShuffleboardTab mainTab;
     boolean useNativePoseEst;
 
     public AprilTagFinder() {
         
 
         //m_detector.addFamily("tag16h5");
         m_detector.addFamily("tag36h11");
         mainTab = Shuffleboard.getTab("main");
     }
 
     public double process(Mat picture, int id) {
         if (picture.empty()) {
             return -1;
         }
 
         AprilTagDetection[] ret = m_detector.detect(picture);
 
         if (ret == null) {
             return -1;
         }
         double center = -1;
         for(int i=0; i<ret.length;i++){
            if(ret[i].getId()==id){
                center = ret[i].getCenterX();
            }
         }
         return center;
     }
 
    
     public void setNativePoseEstimationEnabled(boolean enabled) {
         this.useNativePoseEst = enabled;
     }
 }