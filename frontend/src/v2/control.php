 <?php
	chdir("/home/stagiaire/catkin_ws");
	if (isset($_GET["untuckArm"])) {
		tuckArms($_GET["untuckArm"]);
	}
	if (isset($_GET["display"]) && $_GET["display"]=== "eyes") {
		displayEyes();
	} else if (isset($_GET["display"]) && isset($_GET["top_left"]) && isset($_GET["bottom_left"]) && isset($_GET["top_right"]) && isset($_GET["bottom_right"])) {
		displayCamera($_GET["top_left"], $_GET["bottom_left"], $_GET["top_right"], $_GET["bottom_right"]);
	}

	function launchScript($commandLine) {
    $tab = [];
    $errcode = -12;
		$output = exec($commandLine, $tab, $errcode);
		if ($errcode == 0) {
			echo $commandLine . " done successfully ";
		} else {
			echo $commandLine ." error " . $errcode . "    ";
		}
	}

	function tuckArms($state) {
		$tucking = ($state == "true") ? "-u" : "-t";
		$cmd = "./arms.sh $tucking";
		launchScript($cmd);
	}

	function displayEyes() {
		launchScript("./displayEyes.sh");
	}

	function displayCamera($top_left_camera, $bottom_left_camera, $top_right_camera, $bottom_right_camera) {
		launchScript("./displayCamera.sh $top_left_camera $bottom_left_camera $top_right_camera $bottom_right_camera");
	}
?>
