function studyLog(e){if("undefined"!=typeof _EXPID){var o=new FormData;o.append("expid",_EXPID),o.append("entry",e),$.ajax("../../api/user_studies/study_logs/",{data:o,cache:!1,contentType:!1,processData:!1,type:"POST",beforeSend:function(e){e.setRequestHeader("RMS-Use-Session","true")}})}}