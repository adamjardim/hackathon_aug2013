function createMenuButtons(){$(function(){$("a",".menu-main-menu").button({icons:{primary:"ui-icon-home"}}),$("a",".menu-account").button({icons:{primary:"ui-icon-gear"}}),$("a",".menu-admin-panel").button({icons:{primary:"ui-icon-wrench"}}),$("a",".menu-study-panel").button({icons:{primary:"ui-icon-person"}}),$("a",".menu-logout").button({icons:{primary:"ui-icon-power"}})})}function createModalPageLoading(){$("#LOADING-DIV")&&$("#LOADING-DIV").remove(),$("body").append('<div id="LOADING-DIV" class="modal"></div>'),$("body").addClass("loading")}function removeModalPageLoading(){$("#LOADING-DIV").remove(),$("body").removeClass("loading")}function createErrorDialog(o){$("#ERROR-DIALOG")&&$("#ERROR-DIALOG").remove();var n='<div id="ERROR-DIALOG"><b>';n+='<span class="ui-icon ui-icon-alert" ',n+='style="float: left; margin: 0 7px 50px 0;"></span>',n+=o,n+="</b></div>",$("body").append(n),$("#ERROR-DIALOG").dialog({title:"Error!",draggable:!1,resizable:!1,dialogClass:"alert",modal:!0,show:"fade",hide:"fade",buttons:{Ok:function(){$(this).dialog("close")}}})}function createSlideshow(){$(function(){$("#slides").slides({preload:!0,preloadImage:"css/images/loading.gif",play:5e3,pause:3500,hoverPause:!0,animationStart:function(){$(".caption").animate({bottom:-35},100)},animationComplete:function(){$(".caption").animate({bottom:0},200)},slidesLoaded:function(){$(".caption").animate({bottom:0},200)}})})}function rosonline(o,n,a,e){var t=new ROSLIB.Ros({url:o+n+":"+a});t.on("connection",function(){e(!0)}),t.on("error",function(){e(!1)})}function logout(){createModalPageLoading();var o=new FormData;o.append("request","destroy_session"),$.ajax("../../api/users/user_accounts/",{data:o,cache:!1,contentType:!1,processData:!1,type:"POST",beforeSend:function(o){o.setRequestHeader("RMS-Use-Session","true")},success:function(){window.location="../"}})}function base64Encode(o){var n="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";o=o.replace(/\r\n/g,"\n");for(var a="",e=0;o.length>e;e++){var t=o.charCodeAt(e);128>t?a+=String.fromCharCode(t):t>127&&2048>t?(a+=String.fromCharCode(192|t>>6),a+=String.fromCharCode(128|63&t)):(a+=String.fromCharCode(224|t>>12),a+=String.fromCharCode(128|63&t>>6),a+=String.fromCharCode(128|63&t))}for(var i,r,c,s,d,u,l,m="",f=0;a.length>f;)i=a.charCodeAt(f++),r=a.charCodeAt(f++),c=a.charCodeAt(f++),s=i>>2,d=(3&i)<<4|r>>4,u=(15&r)<<2|c>>6,l=63&c,isNaN(r)?u=l=64:isNaN(c)&&(l=64),m=m+n.charAt(s)+n.charAt(d)+n.charAt(u)+n.charAt(l);return m}