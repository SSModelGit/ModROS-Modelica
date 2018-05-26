within ModROS.Blocks;

block ROS_Sampler "Samples from a ROS socket, via the external C function"
  extends Modelica.Blocks.Interfaces.DiscreteMIMO;
  parameter Integer portNumber = 9090 annotation(Dialog(group = "External Connection Parameters"));
  parameter String hostName = "localhost" annotation(Dialog(group = "External Connection Parameters"));
equation
  when sampleTrigger then
    y = ModROS.Blocks.Internal.ROS_Socket_Call(time, portNumber, hostName, nin, nout, u);
  end when annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROS_Sampler;
