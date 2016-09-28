// Custom
(function ($) {

  // Mobile helper
  // @codekit-prepend "vendor/mobile-helper.js";
  // @codekit-prepend "vendor/instantclick.js";

  // Bootstrap JS stack
  // @codekit-prepend "vendor/bootstrap/transition.js";
  // @codekit-prepend "vendor/bootstrap/collapse.js";
  // @codekit-prepend "vendor/bootstrap/dropdown.js";
  // @codekit-prepend "vendor/bootstrap/affix.js";

  // MixItUp!
  // @codekit-prepend "vendor/jquery.mixitup.js";

  // Scroll Top
  // @codekit-prepend "vendor/jquery.scrollTo.js";
  // @codekit-prepend "vendor/jquery.localScroll.js";


  function offcanvas() {
    // if (matchMedia('(max-width: 769px)').matches) {
      $('#toggle').click(function () {
        $(this).toggleClass('active-toggler');
        $(".row-offcanvas").toggleClass("active");
      });
      $('#main').click(function () {
        $("#toggle").removeClass("active-toggler");
        $(".row-offcanvas").removeClass("active");
      });
    // }
  }

  function animatedHeader() {
    // http://stackoverflow.com/questions/6713324/how-to-shrink-navigation-menu-when-scrolling-down
    $(function(){
      $('#navigation').data('size','big');
    });
    $(window).scroll(function(){
      var $nav = $('#navigation');
      if ($('body').scrollTop() > 0) {
        if ($nav.data('size') == 'big') {
          $nav.data('size','small').stop().animate({
            padding: "0px 0px"
          }, 300);
        }
      } else {
        if ($nav.data('size') == 'small') {
          $nav.data('size','big').stop().animate({
            padding: "50px 0px"
          }, 300);
        }
      }
    });
  }

  function gridFilter() {
    $('#portfolio-grid').mixitup();
  }

  function scrollTop() {
    $('#scroll-top').click(function () {
      $('html, body').animate({
        scrollTop: 0
      }, 800);
    });
  }
  // Other functions...

  function init() {
    offcanvas();
    animatedHeader();
    gridFilter();
    scrollTop();
    // More function calls...
  }

  init();

}(jQuery));
