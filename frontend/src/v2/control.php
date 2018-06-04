 <?php
	//line to change to your catkin workspace
	chdir("/home/stagiaire/catkin_ws");
	if (isset($_GET["untuckArm"])) {
		tuckArms($_GET["untuckArm"]);
	}
	if (isset($_GET["display"])) {
		if ($_GET["display"]=== "eyes") {
			displayEyes();
		} else if ($_GET["display"] == "camera" && isset($_GET["camsList"])) {
			$camerasList = json_decode($_GET["camsList"]);
			displayCamera($camerasList[0], $camerasList[1], $camerasList[2], $camerasList[3]);
		}
	}
	if (isset($_GET["changeState"])) {
		if ($_GET["changeState"] == "disable") {
			launchScript("./controlRobot.sh -d");
		} else if ($_GET["changeState"] == "enable") {
			launchScript("./controlRobot.sh -e");
		} else if ($_GET["changeState"] == "reset") {
			launchScript("./controlRobot.sh -r");
		}
	}

	function launchScript($commandLine) {
    $tab = [];
    $errcode = -12;
		$output = exec($commandLine, $tab, $errcode);
		if ($errcode == 0) {
			echo $commandLine . " done successfully ";
		} else {
			echo $commandLine ." error " . $errcode;
		}
	}

	function tuckArms($state) {
		$tucking = ($state == "true") ? "-u" : "-t";
		$cmd = "./controlRobot.sh $tucking";
		launchScript($cmd);
	}

	function displayEyes() {
		launchScript("./controlRobot.sh -y");
	}

	function displayCamera($top_left_camera, $bottom_left_camera, $top_right_camera, $bottom_right_camera) {
		launchScript("./controlRobot.sh -c $top_left_camera $bottom_left_camera $top_right_camera $bottom_right_camera");
	}
?>
