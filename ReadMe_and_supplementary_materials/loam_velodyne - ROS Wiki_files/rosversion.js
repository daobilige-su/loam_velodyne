// version.js, based on SeeSaw 1.0
function Version(sections) {
  var dotversion = ".version.";

  // Tag shows unless already tagged
  $.each(sections.show, function(index, value) {
    $("span" + dotversion + value).not(".versionshow,.versionhide")
                                  .filter(".hidepart")
                                  .addClass("versionshow")
                                  .end()
                                  .filter(".showpart")
                                  .addClass("versionhide");
  });
  $.each(sections.show, function(index, value) {
    $("div" + dotversion + value).not(".versionshow,.versionhide")
                                 .addClass("versionshow");
  });

  // Tag hides unless already tagged
  $.each(sections.hide, function(index, value) {
    $("span" + dotversion + value).not(".versionshow,.versionhide")
                                  .filter(".showpart")
                                  .addClass("versionshow")
                                  .end()
                                  .filter(".hidepart")
                                  .addClass("versionhide");
  });
  $.each(sections.hide, function(index, value) {
    $("div" + dotversion + value).not(".versionshow,.versionhide")
                                 .addClass("versionhide");
  });

  // Show or hide according to tag
  $(".versionshow").removeClass("versionshow").filter("span").show().end().filter("div").show();
  $(".versionhide").removeClass("versionhide").filter("span").hide().end().filter("div").hide();

  $(".rosversion_name").text(sections.target_ros_distro);
}

function getURLParameter(name) {
  return decodeURIComponent(
    (
      new RegExp('[?|&]' + name + '=' + '([^&;]+?)(&|#|;|$)').exec(location.search) || [,""]
    )[1].replace(/\+/g, '%20')) || null;
}

function toggleDocStatus()
{
  if ($("#doc_status").is(":hidden")) {
    $("#doc_status").slideDown();
  } else {
    $("#doc_status").slideUp();
  }
}

$(document).ready(function() {
  var activedistro = "jade"; //CHANGE THIS LINE TO CHANGE THE DISTRO DISPLAYED BY DEFAULT
  var url_distro = getURLParameter('distro');
  if (url_distro) {
    activedistro=url_distro;
  }
  // Make the $ROS_DISTRO replacement work by wrapping it in a span. This is
  // necessary vs. MoinMoin macros because macros are not expanded inside of
  // code blocks, where this replacement is most useful. Using a function for
  // the replacement allows supporting escapes, so that the following transformations
  // are done:
  //   $ROS_DISTRO -> hydro, indigo, etc.
  //   \$ROS_DISTRO -> $ROS_DISTRO
  //   \\$ROS_DISTRO -> \$ROS_DISTRO
  $("#page p:contains($ROS_DISTRO), #page pre:contains($ROS_DISTRO)").each(function() {
    $(this).html($(this).html().replace(/\\?\$ROS_DISTRO/g,
      function(match) {
        if (match[0] == "\\") {
          return "$ROS_DISTRO";
        } else {
          return "<span class=\"rosversion_name\">$ROS_DISTRO</span>";
        }
      })
    );
  });
  $("div.version").hide();
  if ($("#"+activedistro).length > 0) {
    $("#"+activedistro).click();
  } else {
    $("#rosversion_selector button:last").click();
  }
  $("input.version:hidden").each(function() {
    var bg = $(this).attr("value").split(":");
    $("div.version." + bg[0]).css("background-color",bg[1]).removeClass(bg[0]);
  });
});
