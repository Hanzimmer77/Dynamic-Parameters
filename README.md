## Dynamic Parameters Of Human Gait

This project creates a WPF application that uses a  **`Kinect sensor`** to track and visualize human movement in real-time. The  **`MainWindow.xaml.cs`** file contains the core logic, which processes Kinect sensor data to calculate and display velocity, swing arc length, and stride length of a tracked person.

It uses the  **`OxyPlot`** library to create live-updating graphs of these measurements over time. The application also classifies the person's activity (standing, walking, or running) based on their velocity and swing length. The **`MainWindow.xaml`** file defines the user interface, which includes a video feed from the Kinect, three graph plots (for velocity, swing arc, and stride length), and text displays for current measurements and activity classification. 

The interface also includes a button to save swing data to a text file. This application could be useful for sports training, physical therapy, or general motion analysis.
