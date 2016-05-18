function toggleExpandable(id) {
  var d = document.getElementById(id);
  d.style.display = (d.style.display != 'none' ? 'none' : '' );
}

function toggleExpandableJenkins(id) {
  toggleExpandable(id);
  var build_status_image = $('#' + id).attr('build_status_image');
  if (!build_status_image) {
    $('#' + id).attr('build_status_image', true);
    $('#' + id + ' a').each(function() {
      var href = this.href;
      var n = href.lastIndexOf('/', href.length - 2);
      if (n != -1) {
        var jobname = href.substring(n +1);
        $('<br />&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://build.ros.org/buildStatus/icon?job=' + jobname + '"/>').insertAfter(this);
      }
    });
  }
}
