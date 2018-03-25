within ModROS.Blocks.Internal;

function ROS_Socket_Call
  extends Modelica.Icons.Function;
  input Real t "time";
  input Integer port "port number";
  input Real query1 "something random";
  input Real query2 "something random - second strand";
  output Real res[2] "output signal";

  external "C" ROS_Socket_Call(t, port, query1, query2, res) annotation(Include = "#include \"ROS_Socket_Call.c\"", IncludeDirectory = "modelica://ModROS/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROS_Socket_Call;
