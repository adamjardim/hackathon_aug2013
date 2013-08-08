/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Adam Jardim
 * @author Takehiro Oyakawa
 */

var TEMPLATEMAKER = TEMPLATEMAKER || {
  REVISION : '1'
};


/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Adam Jardim
 * @author Takehiro Oyakawa
 */

/**
* An Maker can display the template maker overlay interface.
*
* @constructor
* @param options - object with following keys:
*   * ros - the main ROS connection
*   * tfClient - the TF client to use
*/
TEMPLATEMAKER.Maker = function(options) {
  options = options || {};
  var ros = options.ros;
  var tfClient = options.tfClient;
  
  // create the save service
  var saveClient = new ROSLIB.Service({
    ros : ros,
    name : '/fake_object_markers/save_template',
    serviceType : 'interactive_world_hackathon/SaveTemplate'
  });
  
  // add the main display div
  var display = document.createElement('div');
  display.id = 'maker';
  var width = 1100;
  var height = 600;
  display.style.width = width + 'px';
  display.style.height = height + 'px';
  display.style.background = '#bc7f7f';
  display.style.visibility = 'hidden';
  display.style.position = 'absolute';
  display.style.top = '30px';
  //display.style.right = ((document.body.clientWidth / 2) - (width/2)) + 'px';
  display.style.left = '50px';
  display.style.zIndex = '100';

  // ros3d div
  var ros3d = document.createElement('div');
  ros3d.id = 'ros3d';
  display.appendChild(ros3d);

  var label = document.createElement('span');
  label.innerHTML = 'Name:';
  display.appendChild(label);
  
  // create the name field
  var name = document.createElement('input');
  display.appendChild(name);
  
  // create the save button
  var save = document.createElement('button');
  save.onclick = function() {
    // send the save request
    var saveName = name.value;
    if (saveName.length === 0) {
      saveName = 'default';
    }
    var request = new ROSLIB.ServiceRequest({
      name : saveName
    });
    saveClient.callService(request, function(result) {});
    display.style.visibility = 'hidden';
  };
  save.innerHTML = 'Save';
  display.appendChild(save);

  var body = document.getElementsByTagName('body')[0];
  body.appendChild(display);
  
  // create the viz
  var viewer = new ROS3D.Viewer({
    divID : 'ros3d',
    width : 1100,
    height : 550,
    antialias : true
  });
  viewer.addObject(new ROS3D.Grid());

  // setup the URDF client
  new ROS3D.UrdfClient({
    ros : ros,
    tfClient : tfClient,
    path : 'http://resources.robotwebtools.org/',
    rootObject : viewer.scene
  });
  
  // add them tables!
  new ROS3D.MarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/create_table_marker',
    rootObject : viewer.scene
  });
  for(var i=1; i<=4; i++) {
    new ROS3D.MarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/leg' + i + '_marker',
      rootObject : viewer.scene
    });
  }
  
  // now go for the objects
  new ROS3D.InteractiveMarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/fake_object_markers/fake_marker_server',
    camera : viewer.camera,
    rootObject : viewer.selectableObjects
  });
};

/**
 * Display the interface.
 */
TEMPLATEMAKER.Maker.prototype.display = function() {
  // start by displaying the element
  var display = document.getElementById('maker');
  display.style.visibility = 'visible';
};
