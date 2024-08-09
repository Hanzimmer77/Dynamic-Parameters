using System;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using Microsoft.Kinect;
using System.Timers;
using OxyPlot;
using OxyPlot.Series;
using OxyPlot.Axes;

namespace dynamic
{
    public partial class MainWindow : Window
    {
        private KinectSensor kinectSensor;
        private WriteableBitmap colorBitmap;
        private byte[] colorPixels;

        // Wrist positions for swing arc calculation
        private List<SkeletonPoint> wristPositions = new List<SkeletonPoint>();
        private const int MaxPositions = 30;

        // Foot positions for stride length calculation
        private List<SkeletonPoint> leftFootPositions = new List<SkeletonPoint>();
        private List<SkeletonPoint> rightFootPositions = new List<SkeletonPoint>();

        private Timer timer;
        private double maxSwingLength = 0.0;
        private List<(DateTime Timestamp, double MaxSwingLength)> swingData = new List<(DateTime, double)>();

        private SkeletonPoint? previousHipPosition = null;
        private DateTime? previousTime = null;
        private double velocity = 0.0;
        private const double smoothingFactor = 0.1;

        private PlotModel velocityPlotModel;
        private LineSeries velocitySeries;
        private PlotModel swingPlotModel;
        private LineSeries swingLineSeries;
        private PlotModel stridePlotModel;
        private LineSeries strideSeries;

        private double[] strideLengths = new double[MaxPositions];
        private int strideIndex = 0;
        private int strideCount = 0;


        private const double standingVelocityThreshold = 0.1; // m/s
        private const double walkingVelocityThreshold = 2.5; // m/s
        private const double runningVelocityThreshold = 10; // m/s
        private const double walkingSwingThreshold = 2.5; // meters
        private const double runningSwingThreshold = 4.5; // meters

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
            Unloaded += MainWindow_Unloaded;

            timer = new Timer(1000); // Timer set to 1 second
            timer.Elapsed += Timer_Elapsed;
            timer.Start();

            InitializeVelocityPlot();
            InitializeSwingPlot();
            InitializeStridePlot();
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensor = KinectSensor.KinectSensors[0];

            if (kinectSensor != null && kinectSensor.Status == KinectStatus.Connected)
            {
                kinectSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                kinectSensor.SkeletonStream.Enable();
                kinectSensor.AllFramesReady += KinectSensor_AllFramesReady;

                colorPixels = new byte[kinectSensor.ColorStream.FramePixelDataLength];
                colorBitmap = new WriteableBitmap(
                    kinectSensor.ColorStream.FrameWidth,
                    kinectSensor.ColorStream.FrameHeight,
                    96.0, 96.0, PixelFormats.Bgr32, null);

                KinectVideo.Source = colorBitmap;

                try
                {
                    kinectSensor.Start();
                }
                catch (IOException)
                {
                    kinectSensor = null;
                }
            }
            else
            {
                //StatusText.Text = "Kinect not connected!";
            }
        }

        private void MainWindow_Unloaded(object sender, RoutedEventArgs e)
        {
            if (kinectSensor != null)
            {
                kinectSensor.Stop();
                kinectSensor = null;
            }

            timer.Stop();
        }

        private void KinectSensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyPixelDataTo(colorPixels);
                    colorBitmap.WritePixels(
                        new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight),
                        colorPixels, colorBitmap.PixelWidth * sizeof(int), 0);
                }
            }

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    Skeleton[] skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    foreach (Skeleton skeleton in skeletons)
                    {
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            CalculateVelocity(skeleton);
                            CalculateSwingArc(skeleton);
                            CalculateStrideLength(skeleton);
                        }
                    }
                }
            }
        }

        private void CalculateVelocity(Skeleton skeleton)
        {
            Joint hip = skeleton.Joints[JointType.HipCenter];

            if (hip.TrackingState == JointTrackingState.Tracked)
            {
                DateTime currentTime = DateTime.Now;

                if (previousTime.HasValue)
                {
                    double deltaTime = (currentTime - previousTime.Value).TotalSeconds;
                    if (deltaTime > 0)
                    {
                        double distance = DistanceBetweenPoints(previousHipPosition.Value, hip.Position);
                        velocity = smoothingFactor * (distance / deltaTime) + (1 - smoothingFactor) * velocity;

                        Dispatcher.Invoke(() =>
                        {
                            VelocityText.Text = $"Velocity: {velocity:F2} m/s";
                            velocitySeries.Points.Add(new DataPoint(DateTimeAxis.ToDouble(currentTime), velocity));
                            velocityPlotModel.InvalidatePlot(true);
                        });
                    }
                }

                previousHipPosition = hip.Position;
                previousTime = currentTime;
            }
        }

        private void CalculateSwingArc(Skeleton skeleton)
        {
            Joint wrist = skeleton.Joints[JointType.WristRight];

            if (wrist.TrackingState == JointTrackingState.Tracked)
            {
                wristPositions.Add(wrist.Position);

                if (wristPositions.Count > MaxPositions)
                {
                    wristPositions.RemoveAt(0);
                }

                double arcLength = CalculateArcLength(wristPositions);
                if (arcLength > maxSwingLength)
                {
                    maxSwingLength = arcLength;
                }
            }
        }

        private double CalculateArcLength(List<SkeletonPoint> positions)
        {
            double length = 0.0;

            for (int i = 1; i < positions.Count; i++)
            {
                length += DistanceBetweenPoints(positions[i - 1], positions[i]);
            }

            return length;
        }

        private void CalculateStrideLength(Skeleton skeleton)
        {
            Joint leftFoot = skeleton.Joints[JointType.FootLeft];
            Joint rightFoot = skeleton.Joints[JointType.FootRight];

            if (leftFoot.TrackingState == JointTrackingState.Tracked)
            {
                leftFootPositions.Add(leftFoot.Position);

                if (leftFootPositions.Count > MaxPositions)
                {
                    leftFootPositions.RemoveAt(0);
                }
            }

            if (rightFoot.TrackingState == JointTrackingState.Tracked)
            {
                rightFootPositions.Add(rightFoot.Position);

                if (rightFootPositions.Count > MaxPositions)
                {
                    rightFootPositions.RemoveAt(0);
                }
            }

            if (leftFootPositions.Count >= 2 && rightFootPositions.Count >= 2)
            {
                double strideLength = CalculateStride(leftFootPositions, rightFootPositions);

                // Store the stride length in the array
                strideLengths[strideIndex] = strideLength;
                strideIndex = (strideIndex + 1) % MaxPositions;
                if (strideCount < MaxPositions) strideCount++;

                // Calculate the average stride length
                double averageStrideLength = CalculateAverageStrideLength();

                Dispatcher.Invoke(() =>
                {
                    StrideText.Text = $"Average Stride Length: {(averageStrideLength * 10):F2} meters";
                    strideSeries.Points.Add(new DataPoint(DateTimeAxis.ToDouble(DateTime.Now), averageStrideLength));
                    stridePlotModel.InvalidatePlot(true);
                });
            }
        }

        private double CalculateStride(List<SkeletonPoint> leftPositions, List<SkeletonPoint> rightPositions)
        {
            double leftDistance = DistanceBetweenPoints(leftPositions[leftPositions.Count - 2], leftPositions[leftPositions.Count - 1]);
            double rightDistance = DistanceBetweenPoints(rightPositions[rightPositions.Count - 2], rightPositions[rightPositions.Count - 1]);

            return (leftDistance + rightDistance) / 2.0;
        }

        private double CalculateAverageStrideLength()
        {
            double total = 0.0;
            for (int i = 0; i < strideCount; i++)
            {
                total += strideLengths[i];
            }
            return total / strideCount;
        }

        private double DistanceBetweenPoints(SkeletonPoint p1, SkeletonPoint p2)
        {
            return Math.Sqrt(
                Math.Pow(p2.X - p1.X, 2) +
                Math.Pow(p2.Y - p1.Y, 2) +
                Math.Pow(p2.Z - p1.Z, 2));
        }

        private void Timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            DateTime timestamp = DateTime.Now;
            double swingLength = maxSwingLength;
            swingData.Add((timestamp, swingLength));

            Dispatcher.Invoke(() =>
            {
                ArcText.Text = $"Max Swing Arc Length: {swingLength:F2} meters at {timestamp:HH:mm:ss}";

                swingLineSeries.Points.Add(new DataPoint(DateTimeAxis.ToDouble(timestamp), swingLength));
                swingPlotModel.InvalidatePlot(true);

                // Classify the activity and update the UI
                string activity = ClassifyActivity(velocity, swingLength);
                ActivityText.Text = $"Activity: {activity}";
            });

            maxSwingLength = 0.0;
        }

        private string ClassifyActivity(double velocity, double swingLength)
        {
            if (velocity < standingVelocityThreshold)
            {
                return "Standing";
            }
            else if (velocity < walkingVelocityThreshold && swingLength < walkingSwingThreshold)
            {
                return "Walking";
            }
            else if (velocity < runningVelocityThreshold && swingLength < runningSwingThreshold)
            {
                return "Running";
            }
            else
            {
                return "Running";
            }
        }

        private void InitializeVelocityPlot()
        {
            velocityPlotModel = new PlotModel { Title = "Velocity Over Time" };

            var dateAxis = new DateTimeAxis
            {
                Position = AxisPosition.Bottom,
                StringFormat = "HH:mm:ss",
                Title = "Time",
                IntervalLength = 80,
                MajorGridlineStyle = LineStyle.Solid,
                MinorGridlineStyle = LineStyle.Dot,
                IntervalType = DateTimeIntervalType.Seconds,
                IsZoomEnabled = false,
                IsPanEnabled = false
            };
            velocityPlotModel.Axes.Add(dateAxis);

            var valueAxis = new LinearAxis
            {
                Position = AxisPosition.Left,
                Title = "Velocity (m/s)",
                MajorGridlineStyle = LineStyle.Solid,
                MinorGridlineStyle = LineStyle.Dot
            };
            velocityPlotModel.Axes.Add(valueAxis);

            velocitySeries = new LineSeries { Title = "Velocity", StrokeThickness = 2, MarkerSize = 3 };
            velocityPlotModel.Series.Add(velocitySeries);

            VelocityPlot.Model = velocityPlotModel;
        }

        private void InitializeSwingPlot()
        {
            swingPlotModel = new PlotModel { Title = "Swing Arc Length Over Time" };

            var dateAxis = new DateTimeAxis
            {
                Position = AxisPosition.Bottom,
                StringFormat = "HH:mm:ss",
                Title = "Time",
                IntervalLength = 80,
                MajorGridlineStyle = LineStyle.Solid,
                MinorGridlineStyle = LineStyle.Dot
            };
            swingPlotModel.Axes.Add(dateAxis);

            var valueAxis = new LinearAxis
            {
                Position = AxisPosition.Left,
                Title = "Swing Length (meters)",
                MajorGridlineStyle = LineStyle.Solid,
                MinorGridlineStyle = LineStyle.Dot
            };
            swingPlotModel.Axes.Add(valueAxis);

            swingLineSeries = new LineSeries { Title = "Swing Length" };
            swingPlotModel.Series.Add(swingLineSeries);

            SwingPlotView.Model = swingPlotModel;
        }

        private void InitializeStridePlot()
        {
            stridePlotModel = new PlotModel { Title = "Stride Length Over Time" };

            var dateAxis = new DateTimeAxis
            {
                Position = AxisPosition.Bottom,
                StringFormat = "HH:mm:ss",
                Title = "Time",
                IntervalLength = 80,
                MajorGridlineStyle = LineStyle.Solid,
                MinorGridlineStyle = LineStyle.Dot
            };
            stridePlotModel.Axes.Add(dateAxis);

            var valueAxis = new LinearAxis
            {
                Position = AxisPosition.Left,
                Title = "Stride Length (meters)",
                MajorGridlineStyle = LineStyle.Solid,
                MinorGridlineStyle = LineStyle.Dot
            };
            stridePlotModel.Axes.Add(valueAxis);

            strideSeries = new LineSeries { Title = "Stride Length" };
            stridePlotModel.Series.Add(strideSeries);

            StridePlot.Model = stridePlotModel;
        }

        private void ShowSwingDataButton_Click(object sender, RoutedEventArgs e)
        {
            string filePath = "SwingData.txt";

            try
            {
                using (StreamWriter writer = File.AppendText(filePath))
                {
                    foreach (var data in swingData)
                    {
                        DateTime timestamp = data.Timestamp;
                        double maxSwingLength = data.MaxSwingLength;

                        writer.WriteLine($"Timestamp: {timestamp}, Max Swing Length: {maxSwingLength:F2} meters");
                    }
                }

                MessageBox.Show("Swing data has been saved to SwingData.txt.", "Data Saved", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"An error occurred while saving swing data: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
    }
}