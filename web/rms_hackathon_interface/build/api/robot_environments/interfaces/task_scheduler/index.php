<?php
/**
 * A task scheduler interface for August Hackthon
 *
 * @author     Jun Ki Lee <jun_ki_lee@brown.edu>
 * @copyright  2013 Brown University
 * @license    BSD -- see LICENSE file
 * @version    June, 3 2013
 * @link       http://ros.org/wiki/rms_interactive_world
 */

/**
 * A static class to contain the interface generate function.
 *
 * @author     Jun Ki Lee <jun_ki_lee@brown.edu>
 * @copyright  2013 Brown University
 * @license    BSD -- see LICENSE file
 * @version    June, 3 2013
 */
class task_scheduler
{
    /**
     * Generate the HTML for the interface. All HTML is echoed.
     * @param robot_environment $re The associated robot_environment object for
     *     this interface
     */
    function generate($re)
    {
        global $title;
        
        // check if we have enough valid widgets
        if (!$streams = $re->get_widgets_by_name('MJPEG Stream')) {
            robot_environments::create_error_page(
                'No MJPEG streams found.',
                $re->get_user_account()
            );
        } else if (!$teleop = $re->get_widgets_by_name('Keyboard Teleop')) {
            robot_environments::create_error_page(
                'No Keyboard Teloperation settings found.',
                $re->get_user_account()
            );
        } else if (!$im = $re->get_widgets_by_name('Interactive Markers')) {
            robot_environments::create_error_page(
                'No Interactive Marker settings found.',
                $re->get_user_account()
            );
        } else if (!$nav = $re->get_widgets_by_name('2D Navigation')) {
            robot_environments::create_error_page(
                'No 2D Navaigation settings found.',
                $re->get_user_account()
            );
        } else if (!$re->authorized()) {
            robot_environments::create_error_page(
                'Invalid experiment for the current user.',
                $re->get_user_account()
            );
        } else {
            // lets create a string array of MJPEG streams
            $topics = '[';
            $labels = '[';
            foreach ($streams as $s) {
                $topics .= "'".$s['topic']."', ";
                $labels .= "'".$s['label']."', ";
            }
            $topics = substr($topics, 0, strlen($topics) - 2).']';
            $labels = substr($labels, 0, strlen($topics) - 2).']';

            // we will also need the map
            $widget = widgets::get_widget_by_table('maps');
            $map = widgets::get_widget_instance_by_widgetid_and_id(
                $widget['widgetid'], $nav[0]['mapid']
            );

            $collada = 'ColladaAnimationCompress/0.0.1/ColladaLoader2.min.js'?>
<!DOCTYPE html>
<html>
<head>
<?php $re->create_head() // grab the header information ?>
<title><?php echo $title?></title>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/threejs/r56/three.min.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/EventEmitter2/0.4.11/eventemitter2.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/<?php echo $collada?>">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/roslibjs/r5/roslib.js"></script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/mjpegcanvasjs/r1/mjpegcanvas.min.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/keyboardteleopjs/r1/keyboardteleop.min.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/ros3djs/r6/ros3d.min.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/EaselJS/0.6.0/easeljs.min.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/ros2djs/r1/ros2d.min.js">
</script>
<script type="text/javascript"
  src="http://cdn.robotwebtools.org/nav2djs/r1/nav2d.min.js">
</script>
<script type="text/javascript"
  src="/api/robot_environments/interfaces/task_scheduler/templatemaker.js">
</script>
<script type="text/javascript">
  //connect to ROS
  var ros = new ROSLIB.Ros({
    url : '<?php echo $re->rosbridge_url()?>'
  });

  ros.on('error', function() {
	writeToTerminal('Connection failed!');
  });

  /**
   * Write the given text to the terminal.
   *
   * @param text - the text to add
   */
  function writeToTerminal(text) {
    var div = $('#terminal');
    div.append('<strong> &gt; '+ text + '</strong><br />');
    div.animate({
      scrollTop : div.prop("scrollHeight") - div.height()
    }, 50);
  }

  /**
   * Load everything on start.
   */
  function start() {
    // create MJPEG streams
    new MJPEGCANVAS.MultiStreamViewer({
      divID : 'video',
      host : '<?php echo $re->get_mjpeg()?>',
      port : '<?php echo $re->get_mjpegport()?>',
      width : 400,
      height : 300,
      topics : <?php echo $topics?>,
      labels : <?php echo $labels?>
    });

    // initialize the teleop
    /*
    new KEYBOARDTELEOP.Teleop({
      ros : ros,
      topic : '<?php echo $teleop[0]['twist']?>',
      throttle : '<?php echo $teleop[0]['throttle']?>'
    });
*/

    // create the main viewer
    var viewer = new ROS3D.Viewer({
      divID : 'scene',
      width :  $(document).width(),
      height : $(document).height(),
      antialias : true
    });
    viewer.addObject(new ROS3D.Grid());

    // setup a client to listen to TFs
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0,
      fixedFrame : '<?php echo $im[0]['fixed_frame'] ?>'
    });
    
    new ROS3D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene,
      topic : '<?php echo $map['topic']?>',
      tfClient : tfClient
    });

    // setup the URDF client
    new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      path : 'http://resources.robotwebtools.org/',
      rootObject : viewer.scene
    });

    // setup the marker clients
    <?php
    foreach ($im as $cur) {?>
      new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '<?php echo $cur['topic'] ?>',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects,
        path : 'http://resources.robotwebtools.org/'
      });
    <?php 
    }
    ?>

    // 2D viewer    
    var navView = new ROS2D.Viewer({
      divID : 'nav',
      width : 400,
      height : 300
    });
    
    NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : navView.scene,
      viewer : navView,
      serverName : '<?php echo $nav[0]['actionserver']?>',
      actionName : '<?php echo $nav[0]['action']?>',
      topic : '<?php echo $map['topic']?>',
      withOrientation : true,
      continuous : <?php echo ($map['continuous'] === 0) ? 'true' : 'false'?>
    });
    /*
    var navigator = null;
    var gridClient = new ROS2D.OccupancyGridClient({
      ros: ros,
      rootObject: navView.scene,
      topic : '<?php echo $map['topic']?>',
    });
    //gridClient.on('change', function() {
      navigator = new NAV2D.Navigator({
        ros: ros,
        serverName: '<?php echo $nav[0]['actionserver']?>',
        actionName: '<?php echo $nav[0]['action']?>',
        topic : '<?php echo $map['topic']?>',
        rootObject: navView.scene,
        withOrientation: true,
      });
    //});*/

    // keep the camera centered at the head
    tfClient.subscribe('/head_mount_kinect_rgb_link', function(tf) {
      viewer.cameraControls.center.x = tf.translation.x;
      viewer.cameraControls.center.y = tf.translation.y;
      viewer.cameraControls.center.z = tf.translation.z;
    });

    // move the overlays
    $('#nav').css({left:($(document).width()-400)+'px'});
    $('#toolbar').css({width:($(document).width()-800)+'px'});
    $('#terminal').css({top:($(document).height()-$('#terminal').height())+'px'});
    $('#scheduler').css({ top: ($(document).height() - 400) + 'px', 
                          left: ($(document).width() - 400) + 'px', 
                        });
    $('#scene-blocker').css({width: $(document).width(), height: $(document).height()});

    $('#scheduler').hide();    
    $(':button').button();
    $(':button').css('font-size', '15px');

    /*
    // create the segment button
     var segment = new ROSLIB.ActionClient({
      ros : ros,
      serverName : '/object_detection_user_command',
      actionName : 'pr2_interactive_object_detection/UserCommandAction'
    });
    $('#segment').button().click(function() {
      var goal = new ROSLIB.Goal({
        actionClient : segment,
        goalMessage : {
          request : 1,
          interactive : false
        }
      });
      goal.send();
      writeToTerminal('Object Detection: Segmenting image');
    });*/
    
    // create the align button
    /*
     var hla = new ROSLIB.ActionClient({
      ros : ros,
      serverName : '/high_level_actions',
      actionName : 'higher_level_actions/HighLevelAction'
    });
    $('#align').button().click(function() {
      var goal = new ROSLIB.Goal({
        actionClient : hla,
        goalMessage : {
          actionType : 'alignTable'
        }
      });
      goal.send();
    }); */
    
    // setup a client to listen to table proximity
    /*
    var tableProximityClient = new ROSLIB.Topic({
      ros : ros,
      name : '/high_level_actions/nearTable',
      messageType : '/higher_level_actions/NearTable'
    });
    // enable or disable the align button depending on table proximity
    tableProximityClient.subscribe(function(msg) {
      if (msg.isNearTable)
      {
        $('#align').button('enable');        
      }
      else
      {
        $('#align').button('disable');
      }
    });
    
    // setup a client to listen to align feedback
    var hlaFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/high_level_actions/feedback',
      messageType : '/higher_level_actions/HighLevelActionFeedback'
    });
    // write status updates to the terminal
    hlaFeedback.subscribe(function(msg) {
      writeToTerminal('Table Alignment: ' + msg.feedback.currentStep);
    });*/
    
    // setup a client to listen to segmentation results
    var segmentationFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/object_detection_user_command/result',
      messageType : '/pr2_interactive_object_detection/UserCommandActionResult'
    });
    // write status updates to the terminal
    segmentationFeedback.subscribe(function(msg) {
      writeToTerminal('Object Detection: ' + msg.status.text);
      writeToTerminal('Object Detection: Action finished');
    });
    
    // setup a client to listen to navigation goals
    var navGoalFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/move_base/goal',
      messageType : '/move_base_msgs/MoveBaseActionGoal'
    });
    // write status updates to the terminal
    navGoalFeedback.subscribe(function(msg) {
      writeToTerminal('Navigation: New goal received (' 
        + msg.goal.target_pose.pose.position.x + ',' 
        + msg.goal.target_pose.pose.position.y + ')');
    });
    
    // setup a client to listen to navigation results
    var navFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/move_base/result',
      messageType : '/move_base_msgs/MoveBaseActionResult'
    });
    // write status updates to the terminal
    navFeedback.subscribe(function(msg) {
      writeToTerminal('Navigation: ' + msg.status.text);
      writeToTerminal('Navigation: Action finished');
    });
      
    // setup a client to listen to pickup goals
    var navGoalFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/object_manipulator/object_manipulator_pickup/goal',
      messageType : '/object_manipulation_msgs/PickupActionGoal'
    });
    // write status updates to the terminal
    navGoalFeedback.subscribe(function(msg) {
      writeToTerminal('Pickup: New goal received');
    });
    
    // setup a client to listen to pickup results
    var pickupFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/object_manipulator/object_manipulator_pickup/result',
      messageType : 'object_manipulation_msgs/PickupActionResult'
    });
    // write status updates to the terminal
    pickupFeedback.subscribe(function(msg) {
      if (msg.result.attempted_grasp_results.length > 0 
        && msg.result.attempted_grasp_results[
        msg.result.attempted_grasp_results.length - 1].result_code === 1)
      {
        writeToTerminal('Pickup: Succeeded');
      }
      else
      {
        writeToTerminal('Pickup: Failed');
      }
      writeToTerminal('Navigation: Action finished');
    });

    // template maker    
    var maker = new TEMPLATEMAKER.Maker({
      ros : ros,
      tfClient : tfClient
    });
    
    $('#show_template_button').button().click(function() {
      maker.display();
    });
    
    // setup a client to listen to scheduler feedbacks
    var schedulerFeedback = new ROSLIB.Topic({
      ros : ros,
      name : '/hackathon_scheduler/status',
      messageType : '/hackathon_scheduler/TaskStatus'
    });
    // write status updates to the terminal
    schedulerFeedback.subscribe(function(msg) {
      if (msg && msg.status) {
        switch(msg.status) {
          case 'executing':
            $('#scene-blocker').show();
            $('.event-entry').each(function() {
              if (msg.startTime == $(this).children('td:first').text()) {
                $(this).addClass('animation');
              }
            });
            writeToTerminal('Event [' + msg.taskName + '] is being executed starting from ' +
                            msg.startTime + 'with a message [' +  msg.message + '].');
            break;
          case 'success': 
            writeToTerminal('Event [' + msg.taskName + '] executed successfully. The event started from ' +
                            msg.startTime + 'with a message [' +  msg.message + '].');
            $('#scene-blocker').hide();
            $('.event-entry').removeClass('animation');            
            break;
          case 'teleop':
            writeToTerminal('Event [' + msg.taskName + '] tells the user to teleop to correct the failure. The task started from ' +
                            msg.startTime + 'with a message [' +  msg.message + '].');
            $('#scene-blocker').hide();
            $('#resume_action_button').show();
            $('#show_schedule_button').hide();
            break;
        }

      }
    });

    // setup a service client to add a schedule to a hackathon scheduler
    var addEventClient = new ROSLIB.Service({
      ros : ros,
      name : '/hackathon_scheduler/addEvent',
      serviceType : 'hackathon_scheduler/AddEvent'
    });

    // setup a service client to retrive a full list of schedules to a hackathon scheduler
    var getScheduleClient = new ROSLIB.Service({
      ros : ros,
      name : '/hackathon_scheduler/getSchedule',
      serviceType : 'hackathon_scheduler/GetSchedule'
    });

    // setup a service client to retrive a full list of schedules to a hackathon scheduler
    var removeEventClient = new ROSLIB.Service({
      ros : ros,
      name : '/hackathon_scheduler/removeEvent',
      serviceType : 'hackathon_scheduler/RemoveEvent'
    });

    // setup a service client to add a schedule to a hackathon scheduler
    var printTemplatesClient = new ROSLIB.Service({
      ros : ros,
      name : '/fake_object_markers/print_templates',
      serviceType : 'fake_object_markers/PrintTemplates'
    });

    var resumeActionClient = new ROSLIB.Service({
      ros : ros,
      name : '/hackathon_scheduler/teleopFinishedService',
      serviceType : 'hackathon_scheduler/TeleopFinishedService',
    });

    // updates the schedules list
    function updateSchedule () {
      $('#scheduleList').text('loading...');
      getScheduleClient.callService({}, function(result) {
        if (result.schedule && $.isArray(result.schedule)) {
          var html = '<table><tr>' + 
                     '<th>Time</th>' + 
                     '<th>Event Name</th>' +
                     '<th>Task Type</th>' +
                     '<th>Template Name</th>' +
                     '<th>Delete</th>' +
                     '</tr>';

          for (i in result.schedule) {
            html = html + '<tr class="event-entry">' + 
                   '<td>' + result.schedule[i].startTime + '</td>' +
                   '<td>' + result.schedule[i].taskName + '</td>' +
                   '<td>' + result.schedule[i].taskType + '</td>' +
                   '<td>' + result.schedule[i].parameters + '</td>' +
                   '<td><a class="deleteButtons" href="javascript:void(0);">delete</a></td></tr>';
          }

          html = html + '</table>';
          $('#scheduleList').html(html);
          $('.deleteButtons').click(function() {            
            var request = new ROSLIB.ServiceRequest({              
              startTime: $($(this).parent().siblings()[0]).text(),
            });
            writeToTerminal('The scheduler is trying to delete an event at ' + request.startTime);
            removeEventClient.callService(request, function(result) {
              if (result.success) {
                writeToTerminal('The schedule deleted the event at ' + request.startTime);
                updateSchedule();
              } else {
                writeToTerminal('The scheduler couldn\'t delete the event at ' + request.startTime);
              }
            });
          });
        }
      });
    };

    function refresh_template_list() {
      $('select[name="templates"]').attr('disabled', 'true');      
      printTemplatesClient.callService({}, function(result) {          
        var html = '';
        if (result.list) {
          for (i in result.list) {
            html = html + '<option value="' + result.list[i] + '">' + result.list[i] +
                   '</option>';
          }
        }
        if (html == '') {
          $('select[name="templates"]').html('none');
        } else {
          $('select[name="templates"]').removeAttr('disabled');
          $('select[name="templates"]').html(html);
        }          
      });
    }

    $('select[name="taskType"]').change(function() {
      if ($(this).val() == 'lunch') {
        refresh_template_list();        
      } else {
        $('select[name="templates"]').attr('disabled', 'true');
      }
    });

    $('#resume_action_button').button().click(function() {
      resumeActionClient.callService({}, function() {
        $('#resume_action_button').hide();
        $('#show_schedule_button').show();
      });
    });

    // setup a popup scheduler window
    $('#show_schedule_button').button().click(function() {
      if (!$('.scheduler').is(':visible')) {
        $('.scheduler').show();
        $('#show_schedule_button').button('option', 'label', 'Close the Scheduler');
        updateSchedule();
      } else {
        $('.scheduler').hide();        
        $('#show_schedule_button').button('option', 'label', 'Show the Scheduler');
      }
    });

    $('#addButton').button().click(function() {
      var request = new ROSLIB.ServiceRequest({ event: {
        taskName: $("input[name='taskName']").val(),
        startTime: $("input[name='startTime']").val(),
        taskType: $("select[name='taskType']").val(),
        parameters: ($("select[name='taskType']").val() == 'lunch' ? 
                     $("select[name='templates']").val() : ''),
      }});

      writeToTerminal("sending an event to the scheduler " +
                      request.event.taskName + " " + request.event.startTime + " " + 
                      request.event.taskType+ " " + request.event.parameters);

      addEventClient.callService(request, function(result) {
        if (result.success) {
          writeToTerminal('The scheduler added an event at ' + request.event.startTime + '.');
          updateSchedule();
        } else {
          writeToTerminal("The scheduler couldn't add the event.");
        }
      });
    });

    // fixes the menu in the floating camera feed
    $('body').bind('DOMSubtreeModified', function() {
    	$('body div:last-child').css('z-index', 750);
    });

    writeToTerminal('Interface initialization complete.');
  }
</script>
</head>
<body onload="start();">
  <div class="mjpeg-widget" id="video"></div>
  <div class="nav-widget" id="nav"></div>
  <div class="toolbar" id="toolbar">
    <img src="../api/robot_environments/interfaces/task_scheduler/img/wpi.png" />
    &nbsp;&nbsp;&nbsp;&nbsp;
    <img src="../api/robot_environments/interfaces/task_scheduler/img/brown.png" /><br><br><br>
    <button id="show_schedule_button">Show the Scheduler</button>
    <button id="resume_action_button" style="display:none">Resume the Action</button>
    &nbsp;&nbsp;
    <button id="show_template_button">Create a Template</button> <br>
  </div>
  <div id="terminal" class="terminal"></div>
  <div id="scene" class="scene"></div>
  <div id="scene-blocker" class="scene-blocker" style="display:none;"></div>
  <div id="scheduler" class="scheduler" style="display:none;">    
    <h2>Scheduler</h2>
    <br>
    <h3>List of Task Events</h3>
    <p id="scheduleList">loading...</p>
    <br>
    <h3>Add an Event</h3>
    Event Name : <input type="text" name="taskName" value=""> <br><br>
    Start Time : <input type="time" name="startTime"> &nbsp; &nbsp; &nbsp;
    Task Type : <select name="taskType"> 
      <option value="medicine">medicine</option>
      <option value="lunch">lunch</option>
      <option value="medicine">medicine</option>
    </select> <br><br>    
    Templates : <select name="templates" disabled> <option>none</option> </select>
    <a href="javascript:refresh_template_list();">refresh</a>
    <br><br>
    <button id="addButton">Add</button>
  </div>
</body>
</html>
<?php
        }
    }
}
