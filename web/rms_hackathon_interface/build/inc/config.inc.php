
<?php
/**
 * RMS Site Settings and Configuration
 *
 * Contains the site settings and configurations for the RMS. This file is
 * auto-generated and should not be edited by hand.
 *
 * @author     Auto Generated via Setup Script
 * @copyright  2013 Russell Toris, Worcester Polytechnic Institute
 * @license    BSD -- see LICENSE file
 * @version    August 6, 2013
 * @package    inc
 * @link       http://ros.org/wiki/rms
 */

// database information
$dbhost = 'localhost';
$dbuser = 'rms';
$dbpass = 'test';
$dbname = 'rms';
$db = mysqli_connect($dbhost, $dbuser, $dbpass, $dbname)
or DIE('Connection has failed. Please try again later.');

// Google Analytics tracking ID -- unset if no tracking is being used.
$googleTrackingID = null;

// site copyright and design information
$copyright = '&copy 2013 Brown & WPI';
$title = 'August Hackathon';
// original site design information
$designedBy = 'Site design by
    <a href="http://users.wpi.edu/~rctoris/">Russell Toris</a>';
