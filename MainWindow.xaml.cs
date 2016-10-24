using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;

using Microsoft.Kinect;
using System.IO;
using System.Speech.Synthesis;
using System.Speech.Recognition;
using System.Timers;
using System.Net.Mail;
using System.Net;
using System.Net.Mime;

using LiveCharts;
using LiveCharts.Configurations;
using System.ComponentModel;
//using System.Windows;
//using System.Windows.Controls;
using System.Windows.Threading;
using WpfApplication1.svmPredict;
namespace WpfApplication1
{

    class JointVA
    {
        public string Name { get; set; }
        public float Height { get; set; }
        public float y { get; set; }
        public float z { get; set; }
        public float Velocity { get; set; }
        public float Acceleration { get; set; }
    }

    class JointPosition
    {
        public float jointHeight { get; set; }
        public float jointHorizontalShift { get; set; }
        public float jointVerticalVelocity { get; set; }
        public float jointHorizontalVelocity { get; set; }
        public DateTime time { get; set; }
        public JointPosition()
        {
            jointHeight = 0;
            jointHorizontalShift = 0;
            jointVerticalVelocity = 0;
            jointHorizontalVelocity = 0;
            time = DateTime.Now;
        }
    }



    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        #region Variables

        float A, B, C, D;
        bool hasInitiateFloorPlane = false;

        private const int FrameCounting = 15;
        private const int JointNumber = 10;
        private const float RenderWidth = 640.0f;
        private const float RenderHeight = 480.0f;
        private const double JointThickness = 3;
        private const double BodyCenterThickness = 10;
        private const double ClipBoundsThickness = 10;
        private readonly Brush centerPointBrush = Brushes.Blue;
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private KinectSensor sensor;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;

        private WriteableBitmap colorBitmap, depthBitmap;
        private DepthImagePixel[] depthPixels;
        private byte[] colorPixels, colorDepthPixels;


        private List<JointPosition>[] currentJoints = new List<JointPosition>[JointNumber];
        private JointPosition[] lastJoint = new JointPosition[JointNumber];

        private float[] lastJointHeights = new float[JointNumber];
        private Queue<float>[] currentJointsHeight = new Queue<float>[JointNumber];
        //private float[] lastJointHorizontalShift = new float[JointNumber];
        //private Queue<float>[] currentJointsHorizontalShift = new Queue<float>[JointNumber];
        private int currentJointsCount;

        private SpeechSynthesizer synthesizer = new SpeechSynthesizer();
        private SpeechRecognitionEngine recognitionEngine = new SpeechRecognitionEngine();
        string[] yesno;
        private int fallen = 0;
        private int eAngle = 0;

        private Timer timer = new Timer(5000);
        private Timer autoAngletimer = new Timer(5000);

        private List<JointVA> items = new List<JointVA>();


        StreamWriter swMyfile = new StreamWriter(new FileStream("rawData.csv", FileMode.Append, FileAccess.Write));
        #endregion
        System.Text.StringBuilder sbToCSV = new System.Text.StringBuilder();

        private string svmMark;


        public class MeasureModel
        {
            public DateTime DateTime { get; set; }
            public float Value { get; set; }
        }
        private double _axisMax;
        private double _axisMin;
        public ChartValues<MeasureModel> ChartValues { get; set; }
        public Func<double, string> DateTimeFormatter { get; set; }

        public double AxisStep { get; set; }

        public double AxisMax
        {
            get { return _axisMax; }
            set
            {
                _axisMax = value;
                OnPropertyChanged("AxisMax");
            }
        }
        public double AxisMin
        {
            get { return _axisMin; }
            set
            {
                _axisMin = value;
                OnPropertyChanged("AxisMin");
            }
        }

        public DispatcherTimer Timer { get; set; }
        public bool IsDataInjectionRunning { get; set; }
        public Random R { get; set; }

        private void RunDataOnClick(object sender, RoutedEventArgs e)
        {
            IsDataInjectionRunning = true;
            if (IsDataInjectionRunning)
            {
                Timer.Stop();
                IsDataInjectionRunning = false;
            }
            else
            {
                Timer.Start();
                IsDataInjectionRunning = true;
            }
        }

        private void TimerOnTick(object sender, EventArgs eventArgs)
        {
            var now = DateTime.Now;

            ChartValues.Add(new MeasureModel
            {
                DateTime = now,
                Value = R.Next(0, 10)
            });

            SetAxisLimits(now);

            //lets only use the last 30 values
            if (ChartValues.Count > 30) ChartValues.RemoveAt(0);
        }

        private void SetAxisLimits(DateTime now)
        {
            AxisMax = now.Ticks + TimeSpan.FromSeconds(1).Ticks; // lets force the axis to be 100ms ahead
            AxisMin = now.Ticks - TimeSpan.FromSeconds(5).Ticks; //we only care about the last 8 seconds
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged(string propertyName = null)
        {
            if (PropertyChanged != null)
                PropertyChanged.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public MainWindow()
        {
            InitializeComponent();

            var mapper = Mappers.Xy<MeasureModel>()
                .X(model => model.DateTime.Ticks)   //use DateTime.Ticks as X
                .Y(model => model.Value);

            //lets save the mapper globally.
            Charting.For<MeasureModel>(mapper);


            //the values property will store our values array
            ChartValues = new ChartValues<MeasureModel>();

            //lets set how to display the X Labels
            DateTimeFormatter = value => new DateTime((long)value).ToString("mm:ss");

            AxisStep = TimeSpan.FromSeconds(1).Ticks;
            SetAxisLimits(DateTime.Now);

            //The next code simulates data changes every 300 ms
            
            IsDataInjectionRunning = false;
            R = new Random();

            DataContext = this;

        }

        // Execute startup tasks
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            //*

            ///------------------------------------------------------------------------
            ///--- Detect the amount of the Kinect sensor -----------------------------
            ///------------------------------------------------------------------------
            // If no sensor is detected, shut down the window
            if (KinectSensor.KinectSensors.Count == 0)
            {
                MessageBox.Show("No Kinects device detected", "Fall Detection");
                System.Windows.Application.Current.Shutdown();
                //return;
            }

            // Look through all the sensors and start the first connected one.
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    sensor = potentialSensor;
                    // Start the sensor
                    try
                    {
                        sensor.Start();
                        // Set angle of Kinect
                        sensor.ElevationAngle = -15;
                    }
                    catch (IOException)
                    { sensor = null; }
                    break;
                }
            }

            ///------------------------------------------------------------------------
            ///--- Prepare for the skeleton image -------------------------------------
            ///------------------------------------------------------------------------
            // Create the drawing group we'll use for drawing skeleton
            drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            SkeletalImage.Source = imageSource;


            ///------------------------------------------------------------------------
            ///--- Initialise array of queues -----------------------------------------
            ///------------------------------------------------------------------------
            // Each queue represents a joint, store last 30 heights of it joint
            for (int i = 0; i < currentJointsHeight.Length; i++)
                currentJointsHeight[i] = new Queue<float>();
            for (int i = 0; i < currentJoints.Length; i++)
                currentJoints[i] = new List<JointPosition>();

           
            currentJointsCount = 0;
            svmMark = "-1";

            TransformSmoothParameters smoothingParam = new TransformSmoothParameters();
            smoothingParam.Smoothing = 0.5f;
            smoothingParam.Correction = 0.5f;
            smoothingParam.Prediction = 0.5f;
            smoothingParam.JitterRadius = 0.05f;
            smoothingParam.MaxDeviationRadius = 0.04f;

            ///------------------------------------------------------------------------
            ///--- 1. Initialise three frames: RGB, Skeleton and Depth ----------------
            ///--- 2. And handle the events of these frames ---------------------------
            ///--- 3. Event handler for timer that counts 5 seconds -------------------
            ///------------------------------------------------------------------------
            if (null != this.sensor)
            {
                // Turn on the color stream to receive colored frames
                sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                sensor.ColorFrameReady += SensorColorFrameReady;

                // Turn on the skeleton stream to receive skeleton frames
                sensor.SkeletonStream.Enable(smoothingParam);
                sensor.SkeletonFrameReady += SensorSkeletonFrameReady;
                sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                sensor.SkeletonStream.EnableTrackingInNearRange = true; 
                // Configure the depth stream to receive depth frames
                // Turn on the depth stream to receive depth frames
                //sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                //configureDepthStream();
                //sensor.DepthFrameReady += SensorDepthFrameReady;

                
                // Event handler for timer that counts 5 seconds
                // 5秒钟到了之后就会调用Elapsed
                //timer.Elapsed += timerElapsedHandler;
            }

            ///------------------------------------------------------------------------
            ///--- Build the files that store position of joints ----------------------
            ///------------------------------------------------------------------------
            //*/
        }

        // Execute shutdown tasks
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            //if (null != this.sensor)
            //{ sensor.Stop(); }
        }

        #region Event handler for window
        // Event handler for switching on and off of skeleton image
        void checkBoxSkelOnlyChanged(object sender, RoutedEventArgs e)
        {
            if (checkBoxSkelOnly.IsChecked.GetValueOrDefault())
            {
                SkeletalImage.Visibility = Visibility.Visible;
            }
            else { 
                SkeletalImage.Visibility = Visibility.Hidden;
            }
        }

        // Event handler for angle change button
        private void angleButton_Click(object sender, RoutedEventArgs e)
        {
            eAngle = Convert.ToInt32(verticalAngle.Text);
            sensor.ElevationAngle = eAngle;
            currentAngleLabel.Content = "Current Angle: "+eAngle+"°";
        }

        private void ImageTypeHandleCheck(object sender, RoutedEventArgs e)
        {
            if (rgbButton.IsChecked == true)
            {
                showRGB();
                hideDepth();
            }
            else if (depthButton.IsChecked == true)
            {
                showDepth();
                hideRGB();
            }
        }

        private void showRGB()
        {
            ColorImage.Visibility = Visibility.Visible;
        }

        private void showDepth()
        {
            DepthImage.Visibility = Visibility.Visible;

        }

        private void hideRGB()
        {
            ColorImage.Visibility = Visibility.Hidden;
        }

        private void hideDepth()
        {
            DepthImage.Visibility = Visibility.Hidden;
        }

        private void HandleCheck(object sender, RoutedEventArgs e)
        {
            svmMark = "1 ";
            svmButton.Content = "fall end";
            svmButton.Background = new SolidColorBrush(Color.FromArgb(0xFF, 0xBB, 0xDE, 0xFD));
        }

        private void HandleUnchecked(object sender, RoutedEventArgs e)
        {
            svmMark = "-1 ";
            svmButton.Content = "fall begin";
            svmButton.Background = new SolidColorBrush(Color.FromArgb(0xFF, 0xFF, 0xB7, 0x4D));
        }

        private void HandleSeat(object sender, RoutedEventArgs e)
        {
            sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
            seatButton.Content = "Current Seated";
            seatButton.Background = new SolidColorBrush(Color.FromArgb(0xFF, 0xBB, 0xDE, 0xFD));
        }

        private void HandleDefault(object sender, RoutedEventArgs e)
        {
            sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
            seatButton.Content = "Current Default";
            seatButton.Background = new SolidColorBrush(Color.FromArgb(0xFF, 0xFF, 0xB7, 0x4D));
        }  
        #endregion

        #region Event handler for Depth stream
        // Initialize the depthstream and prepare for the color afterwards
        private void configureDepthStream()
        {
            int frameLength = sensor.DepthStream.FramePixelDataLength;//frameLength = 307200=640*480
            
            // Allocate space to put the depth pixels we'll receive
            depthPixels = new DepthImagePixel[frameLength];
            // Allocate space to put the color pixels we'll create
            colorDepthPixels = new byte[frameLength * sizeof(int)]; //sizeof(int)=4
            
            // Bitmap that will be displayed
            depthBitmap = new WriteableBitmap(sensor.DepthStream.FrameWidth, sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            DepthImage.Source = depthBitmap;
        }

        // Event handler for sensor's DepthFrameReady event
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(depthPixels);

                    // Get the min and max depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;

                    // Convert the depth to RGB
                    int depthPixelIndex = 0;
                    for (int i = 0; i < depthPixels.Length; ++i) //depthPixels.Length = 307200
                    {
                        // Get the depth for this pixel
                        short depth = depthPixels[i].Depth;

                        byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                        // Write out blue byte
                        colorDepthPixels[depthPixelIndex++] = intensity;

                        // Write out green byte
                        colorDepthPixels[depthPixelIndex++] = intensity;

                        // Write out red byte                        
                        colorDepthPixels[depthPixelIndex++] = intensity;

                        depthPixelIndex++;
                    }

                    // Write the pixel data into our bitmap to update it
                    // --sourceRect: The rectangle of the WriteableBitmap to update.
                    // --pixels: The pixel array used to update the bitmap.
                    // --stride: The stride of the update region in pixels.
                    // --offset: The input buffer offset.
                    // http://www.360doc.com/content/13/0424/23/11482448_280721226.shtml
                    depthBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                        colorDepthPixels,
                        depthBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }
        }

        #endregion

        #region Event handler for Color stream
        // Event handler for sensor's ColorFrameReady event
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    colorPixels = new byte[colorFrame.PixelDataLength]; 
                    //colorFrame.PixelDataLength = 1228800 = 720*480*4

                    colorFrame.CopyPixelDataTo(colorPixels);

                    colorBitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96, 96, PixelFormats.Bgr32, null);
                    colorBitmap.WritePixels(new Int32Rect(0, 0, colorFrame.Width, colorFrame.Height), colorPixels, colorFrame.Width * 4, 0);
                    ColorImage.Source = colorBitmap;
                }
            }
        }
        
        #endregion

        #region Event handler for Skeleton stream
        // Event handler for sensor's SkeletonFrameReady event
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            // select the first person detected to detect fall and draw the skeleton of it
            Skeleton[] skeletons = new Skeleton[0];

            #region Detect Fall
            
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (!hasInitiateFloorPlane)
                {
                    if (skeletonFrame.FloorClipPlane.Item1 != 0)
                    {
                        A = skeletonFrame.FloorClipPlane.Item1;
                        B = skeletonFrame.FloorClipPlane.Item2;
                        C = skeletonFrame.FloorClipPlane.Item3;
                        D = skeletonFrame.FloorClipPlane.Item4;
                        hasInitiateFloorPlane = true;
                        sbToCSV.Append("A:"+A+" B:"+B+" C:"+C+" D:"+D);
                    }
                }

                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];//skeletonFrame.SkeletonArrayLength = 6
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    // set the count to calculate time
                    if (currentJointsCount == 30)
                        currentJointsCount = 0;
                    currentJointsCount++;

                    foreach (Skeleton skeleton in skeletons)
                    {
                        // determine all of the tracked joints
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                            detectFall(skeletonFrame, skeleton);
                    }
                    
                }
            }

            #endregion

            #region Draw Skeleton
            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));
                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            centerPointBrush,
                            null,
                            SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }
                // Prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
            #endregion
        }

        #region draw sekeleton functions
        // Draws skeleton's bones and joints
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints and output the joints' position
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;
                 
                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = trackedJointBrush; 
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                { drawBrush = inferredJointBrush; }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        // Draws a bone line between two joints
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            { return; }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            { return; }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            { drawPen = this.trackedBonePen; }

            drawingContext.DrawLine(drawPen, SkeletonPointToScreen(joint0.Position), SkeletonPointToScreen(joint1.Position));
        }

        // Maps a SkeletonPoint to lie within our render space and converts to Point
        private System.Windows.Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new System.Windows.Point(depthPoint.X, depthPoint.Y);
        }
        
        // Draws indicators to show which edges are clipping skeleton data
        private void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        #endregion

        #region fall detection
        // Makes a floor plane and uses the position of the head to calculate the length of a vector normal to the plane
        private void detectFall(SkeletonFrame sframe, Skeleton skeleton)
        {
            int fallCounter = 0;
            int trackedJoints = 0;

            // Array of joints we want to take into account
            Joint[] joints = new Joint[] {
                skeleton.Joints[JointType.Head],
                skeleton.Joints[JointType.ShoulderCenter],
                skeleton.Joints[JointType.ShoulderLeft],
                skeleton.Joints[JointType.ShoulderRight]
            };

            Joint[] allJoints = new Joint[] {
                skeleton.Joints[JointType.HipCenter],//0
                skeleton.Joints[JointType.Head],//3
                skeleton.Joints[JointType.ShoulderLeft],//4
                skeleton.Joints[JointType.HandLeft],//7
                skeleton.Joints[JointType.ShoulderRight],//8
                skeleton.Joints[JointType.HandRight],//11
                skeleton.Joints[JointType.KneeLeft],//14
                skeleton.Joints[JointType.KneeRight]//17
            };

            Joint[] seatJoints = new Joint[] {
                skeleton.Joints[JointType.ShoulderCenter],//2 
                skeleton.Joints[JointType.Head],//3
                skeleton.Joints[JointType.ShoulderLeft],//4
                skeleton.Joints[JointType.ElbowLeft],//5
                skeleton.Joints[JointType.WristLeft],//6
                skeleton.Joints[JointType.HandLeft],//7
                skeleton.Joints[JointType.ShoulderRight],//8
                skeleton.Joints[JointType.ElbowRight],//9
                skeleton.Joints[JointType.WristRight],//10
                skeleton.Joints[JointType.HandRight],//11
            };

            // Pre processing the CSV file, list and graph
            sbToCSV.Clear();
            sbToCSV.Append(svmMark); 
            
            jointInformationList.ItemsSource = null;
            items.Clear();

            // Check if a joint is in a 'falling motion'
            for (int i = 0; i < seatJoints.Length; i++)
            //int i = 0;
            //foreach (Joint currentJoint in skeleton.Joints)
            {

                if (seatJoints[i].TrackingState == JointTrackingState.Tracked)
                {
                    fallCounter += checkJoint(sframe, seatJoints[i], i);
                    trackedJoints++;
                    
                }
               // else sbToCSV.Append(",,,");
                
            }

            // Follow up processing the CSV file, list and graph
            jointInformationList.ItemsSource = items;

            swMyfile.WriteLine(sbToCSV);
            swMyfile.Flush();

            // Start emergency procedure when we detect a falling person
            if (trackedJoints > 0 && fallCounter == trackedJoints)
            {
                fallen++;
                label.Content = "Fall Detected";
                label.Foreground = Brushes.Red;
                if (fallen == 1)
                {
                    saveImage();
                    //setupSpeechRecognition();
                    //synthesizer.Speak("Do you need assistance?");
                    timer.AutoReset = false;
                    timer.Start();
                }
            }
        }

        private int checkJoint(SkeletonFrame sframe, Joint joint, int n)
        {
            DateTime currentTime = DateTime.Now;

            float x = joint.Position.X;
            float y = joint.Position.Y;
            float z = joint.Position.Z;

            float verticalDistance = distanceToFloorPlane(sframe, joint);
            float horizontalDistance = horizontalShiftToFloorPlane(sframe, joint);
            float verticalShift = 0, horizontalShift = 0, lastSecondAverageVerticalVelocity = 0, lastSecondAverageHorizontalVelocity = 0,verticalAcceleration = 0,horizontalAcceleration=0;
            
            // Dequeue when the queue has size 30(FrameCounting)
            if (currentJoints[n].Count == FrameCounting)
                currentJoints[n].RemoveAt(0);
            //float lastSecondAverageVelocity = heightDifferenceTotal * FrameCounting / currentJointsHeight[n].Count;
            double timeSpan = 0;
            if (currentJoints[n].Count > 1)
            {
                timeSpan = Math.Floor((currentTime - currentJoints[n][0].time).TotalMilliseconds * 10);
                if (timeSpan < 20000 && timeSpan > 0)
                {
                    verticalShift = verticalDistance - currentJoints[n][0].jointHeight;
                    horizontalShift = horizontalDistance - currentJoints[n][0].jointHorizontalShift;
                    lastSecondAverageVerticalVelocity = verticalShift * 100000 / (float)timeSpan;
                    lastSecondAverageHorizontalVelocity = horizontalShift * 100000 / (float)timeSpan;
                    verticalAcceleration = lastSecondAverageVerticalVelocity - currentJoints[n][0].jointVerticalVelocity * 100000 / (float)timeSpan;
                    horizontalAcceleration = lastSecondAverageHorizontalVelocity - currentJoints[n][0].jointHorizontalVelocity * 100000 / (float)timeSpan;
                    //sbToCSV.Append(verticalDistance + ",");
                    //sbToCSV.Append(lastSecondAverageVerticalVelocity + ",");
                    //sbToCSV.Append(lastSecondAverageHorizontalVelocity + ",");
                    //sbToCSV.Append(verticalAcceleration + ",");
                    //sbToCSV.Append(horizontalAcceleration + ",");

                    switch ((int)joint.JointType)
                    {
                        case 2:
                            sbToCSV.Append(" 1:"+verticalDistance);
                            sbToCSV.Append(" 2:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 3:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 4:" + verticalAcceleration);
                            sbToCSV.Append(" 5:" + horizontalAcceleration);
                            break;
                        case 3:
                            sbToCSV.Append(" 6:" + verticalDistance);
                            sbToCSV.Append(" 7:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 8:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 9:" + verticalAcceleration);
                            sbToCSV.Append(" 10:" + horizontalAcceleration);
                            break;
                        case 4:
                            sbToCSV.Append(" 11:" + verticalDistance);
                            sbToCSV.Append(" 12:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 13:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 14:" + verticalAcceleration);
                            sbToCSV.Append(" 15:" + horizontalAcceleration);
                            break;
                        case 5:
                            sbToCSV.Append(" 16:" + verticalDistance);
                            sbToCSV.Append(" 17:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 18:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 19:" + verticalAcceleration);
                            sbToCSV.Append(" 20:" + horizontalAcceleration);
                            break;
                        case 6:
                            sbToCSV.Append(" 21:" + verticalDistance);
                            sbToCSV.Append(" 22:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 23:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 24:" + verticalAcceleration);
                            sbToCSV.Append(" 25:" + horizontalAcceleration);
                            break;
                        case 7:
                            sbToCSV.Append(" 26:" + verticalDistance);
                            sbToCSV.Append(" 27:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 28:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 29:" + verticalAcceleration);
                            sbToCSV.Append(" 30:" + horizontalAcceleration);
                            break;
                        case 8:
                            sbToCSV.Append(" 31:" + verticalDistance);
                            sbToCSV.Append(" 32:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 33:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 34:" + verticalAcceleration);
                            sbToCSV.Append(" 35:" + horizontalAcceleration);
                            break;
                        case 9:
                            sbToCSV.Append(" 36:" + verticalDistance);
                            sbToCSV.Append(" 37:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 38:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 39:" + verticalAcceleration);
                            sbToCSV.Append(" 40:" + horizontalAcceleration);
                            break;
                        case 10:
                            sbToCSV.Append(" 41:" + verticalDistance);
                            sbToCSV.Append(" 42:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 43:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 44:" + verticalAcceleration);
                            sbToCSV.Append(" 45:" + horizontalAcceleration);
                            break;
                        case 11:
                            sbToCSV.Append(" 46:" + verticalDistance);
                            sbToCSV.Append(" 47:" + lastSecondAverageVerticalVelocity);
                            sbToCSV.Append(" 48:" + lastSecondAverageHorizontalVelocity);
                            sbToCSV.Append(" 49:" + verticalAcceleration);
                            sbToCSV.Append(" 50:" + horizontalAcceleration);
                            break;
                    }
                    
                    //sbToCSV.Append((int)joint.JointType + 1);
                    //sbToCSV.Append(":VV " + lastSecondAverageVerticalVelocity + ",HV " + lastSecondAverageHorizontalVelocity + ",TS " + timeSpan + " || ");
                }
                //else
                    //sbToCSV.Append(",,,");

                /*if ((int)joint.JointType == 3)
                {
                    ChartValues.Add(new MeasureModel { DateTime = currentTime, Value = lastSecondAverageVerticalVelocity });
                    SetAxisLimits(currentTime);
                    //lets only use the last 30 values
                    if (ChartValues.Count > 200) ChartValues.RemoveAt(0);
                }*/
                
            }
            //else
                //sbToCSV.Append(",,,");

            currentJoints[n].Add(new JointPosition() { jointHeight = verticalDistance, jointHorizontalShift = horizontalDistance, jointVerticalVelocity = lastSecondAverageVerticalVelocity, jointHorizontalVelocity = lastSecondAverageHorizontalVelocity, time = currentTime });

            // Add items to the graph
            //items.Add(new JointVA() { Name = joint.JointType.ToString(), Height = verticalDistance, y = y, z = z, Velocity = lastSecondAverageVerticalVelocity, Acceleration = lastSecondAverageHorizontalVelocity });

            // If joint is below a certain altitude and the acceleration is faster than a certain threshold the tracked joint is in a 'falling motion'
            if (verticalDistance <= 0.80 && lastSecondAverageVerticalVelocity > 1.0)
            {
                return 1;
            }

            return 0;

            #region previous version of fall detection
            /*
            // Add new value to queue (last position of Y minus the current position of Y)
            dPosY[n].Enqueue(lastPosY[n] - y);
            lastPosY[n] = y;

            // Add last 10 (or the amount we have in our queue) together
            float dPosYTotal = 0.0f;
            foreach (float dPosYStep in dPosY[n])
                dPosYTotal += dPosYStep;

            // Divide to get the average acceleration of joint
            float dPosYAvg = dPosYTotal / dPosY[n].Count;

            // If joint is below a certain altitude and the acceleration is faster than a certain threshold the tracked joint is in a 'falling motion'
            if (distanceToFloorPlane(sframe,joint) <= 0.80 && dPosYAvg > 0.05)
            {
                return 1;
            }

            return 0;
            */
            #endregion

        }

        private float distanceToFloorPlane(SkeletonFrame sframe, Joint joint)
        {
            float x = joint.Position.X;
            float y = joint.Position.Y;
            float z = joint.Position.Z;

            // Calculate distance of joint with respect to the floorplane
            float num = A * x + B * y + C * z + D;
            float denum = A * A + B * B + C * C;
            //sbToCSV.Append("verticalDistance: A "+A+" B "+B+" C "+C+" D "+D+" x "+x+" y "+y+" z "+z+" ");

            return num / (float)Math.Sqrt(denum);
        }

        private float horizontalShiftToFloorPlane(SkeletonFrame sframe, Joint joint)
        {
            float x = joint.Position.X;
            float y = joint.Position.Y;
            float z = joint.Position.Z;

            // Calculate distance of joint with respect to the floorplane
            float num = A * x + B * y + C * z + D;
            float denum = A * A + B * B + C * C;

            float x2 = x - A * num / denum;
            float z2 = z - C * num / denum;

            return (float)Math.Sqrt(x2 * x2 + z2 * z2);
        }

        #endregion

        #endregion

        #region Speech recognition
        // Speech recognition
        private void setupSpeechRecognition()
        {
            recognitionEngine.SetInputToDefaultAudioDevice();
            yesno = new string[2] { "Yes", "No" };
            foreach (string s in yesno)
            {
                recognitionEngine.RequestRecognizerUpdate();
                recognitionEngine.LoadGrammar(new Grammar(new GrammarBuilder(s)));
            }
            recognitionEngine.SpeechRecognized += recognitionEngine_SpeechRecognized;
            recognitionEngine.SpeechRecognitionRejected += recognitionEngine_SpeechRecognitionRejected;
            recognitionEngine.RecognizeAsync(RecognizeMode.Single);
        }

        void recognitionEngine_SpeechRecognitionRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
            string word = yesno[0] + " or " + yesno[1];
            foreach (RecognizedPhrase a in e.Result.Alternates)
            { word = a.Text; }
            synthesizer.Speak("I did not understand that. Did you mean " + word);
        }

        void recognitionEngine_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            string word = e.Result.Text;
            if (e.Result.Text == "No")
            {
                fallen = 0;
                label.Content = "No Fall Detected";
                label.Foreground = Brushes.Lime;
                timer.Stop();
            }
            else if (e.Result.Text == "Yes")
            {
                createMessage();
            }
        }

        // Save snapshot of falling person
        private void saveImage()
        {
            string path = "../Snapshots/Fall.jpg";
            FileStream fs = new FileStream(path, FileMode.Create);
            RenderTargetBitmap bmp = new RenderTargetBitmap((int)ColorImage.ActualWidth,
                (int)ColorImage.ActualHeight, 1 / 96, 1 / 96, PixelFormats.Pbgra32);
            bmp.Render(ColorImage);
            BitmapEncoder encoder = new JpegBitmapEncoder();//new TiffBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bmp));
            encoder.Save(fs);
            fs.Close();
        }

        // Send an email to emergency address containing snapshot so the receiver can judge the situation
        public void createMessage()
        {
            string file = "../Snapshots/Fall.jpg";

            MailMessage message = new MailMessage("meganchang1111@gmail.com", "jing15@connect.hku.hk", "Someone is hurt!!", "Assistance is needed, see the attachment");
            Attachment data = new Attachment(file, MediaTypeNames.Application.Octet);
            ContentDisposition disposition = data.ContentDisposition;
            disposition.CreationDate = File.GetCreationTime(file);
            disposition.ModificationDate = File.GetLastWriteTime(file);
            disposition.ReadDate = File.GetLastAccessTime(file);

            message.Attachments.Add(data);

            var client = new SmtpClient("smtp.gmail.com", 587)
            {
                Credentials = new NetworkCredential("meganchang1111@gmail.com ", "jingjing.230031"),
                EnableSsl = true
            };

            try
            { client.Send(message); }
            catch (Exception ex)
            {
                Console.WriteLine("Exception caught in CreateMessageWithAttachment(): {0}",
                      ex.ToString());
            }

            data.Dispose();
        }

        void timerElapsedHandler(object sender, ElapsedEventArgs e)
        {
            recognitionEngine.Dispose();
            synthesizer.Speak("The assistance is underway!");
            createMessage();
            MessageBox.Show("The assistance is underway!", "Don't panic");
        }

        #endregion

    }
}
