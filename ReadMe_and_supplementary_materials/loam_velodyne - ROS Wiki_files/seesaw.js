// seesaw.js for SeeSaw 1.0
function seeSaw(sections,speed,seesaw) {
  var dotseesaw = seesaw ? ".seesaw." : ".";

  // Tag toggles
  $.each(sections.toggle, function() {
    $("span.seesaw." + this + ",div" + dotseesaw + this).each(function() {
      if ($(this).css("display") == "none") {
        $(this).addClass("seesawshow");
      }
    });
  });
  $.each(sections.toggle, function() {
    $("span.seesaw." + this + ",div" + dotseesaw + this).each(function() {
      if ($(this).css("display") != "none") {
        $(this).addClass("seesawhide");
      }
    });
  });

  // Tag shows unless already tagged
  $.each(sections.show, function() {
    $("span.seesaw." + this).not(".seesawshow,.seesawhide").filter(".hidepart").addClass("seesawshow").end().filter(".showpart").addClass("seesawhide");
  });
  $.each(sections.show, function() {
    $("div" + dotseesaw + this).not(".seesawshow,.seesawhide").addClass("seesawshow");
  });

  // Tag hides unless already tagged
  $.each(sections.hide, function() {
    $("span.seesaw." + this).not(".seesawshow,.seesawhide").filter(".showpart").addClass("seesawshow").end().filter(".hidepart").addClass("seesawhide");
  });
  $.each(sections.hide, function() {
    $("div" + dotseesaw + this).not(".seesawshow,.seesawhide").addClass("seesawhide");
  });

  // Show or hide according to tag
  $(".seesawshow").removeClass("seesawshow").filter("span").show().end().filter("div").show(speed);
  $(".seesawhide").removeClass("seesawhide").filter("span").hide().end().filter("div").hide(speed);
}

$(document).ready(function() {
  $("div.seesaw").not(".show").hide();
  $("input.seesaw:hidden").each(function() {
    var bg = $(this).attr("value").split(":");
    $("div.seesaw." + bg[0]).css("background-color",bg[1]).removeClass(bg[0]);
  });
});
