// ===== Mobile Navigation Toggle =====
document.addEventListener('DOMContentLoaded', () => {
  const hamburger = document.querySelector('.hamburger');
  const navLinks = document.querySelector('.nav-links');

  if (hamburger) {
    hamburger.addEventListener('click', () => {
      navLinks.classList.toggle('open');
    });
  }

  // Close mobile nav when clicking a link
  document.querySelectorAll('.nav-links a').forEach(link => {
    link.addEventListener('click', () => {
      navLinks.classList.remove('open');
    });
  });

  // ===== Active Nav Link =====
  const currentPage = window.location.pathname.split('/').pop() || 'index.html';
  document.querySelectorAll('.nav-links a').forEach(link => {
    const href = link.getAttribute('href').split('/').pop();
    if (href === currentPage) {
      link.classList.add('active');
    }
  });

  // ===== Hero Carousel (Coverflow) =====
  const carousel = document.querySelector('.hero-carousel');
  if (carousel) {
    const slides = carousel.querySelectorAll('.carousel-slide');
    const dotsContainer = carousel.querySelector('.carousel-dots');
    const prevBtn = carousel.querySelector('.carousel-prev');
    const nextBtn = carousel.querySelector('.carousel-next');
    const total = slides.length;
    let current = 0;
    let autoPlay;

    // Create dots
    slides.forEach((_, i) => {
      const dot = document.createElement('button');
      dot.classList.add('carousel-dot');
      if (i === 0) dot.classList.add('active');
      dot.setAttribute('aria-label', 'Go to slide ' + (i + 1));
      dot.addEventListener('click', () => { stopAuto(); goTo(i); startAuto(); });
      dotsContainer.appendChild(dot);
    });

    const dots = dotsContainer.querySelectorAll('.carousel-dot');

    function updatePositions() {
      slides.forEach((slide, i) => {
        const offset = (i - current + total) % total;
        if (offset === 0) {
          slide.setAttribute('data-pos', 'center');
        } else if (offset === total - 1) {
          slide.setAttribute('data-pos', 'left');
        } else if (offset === 1) {
          slide.setAttribute('data-pos', 'right');
        } else {
          slide.setAttribute('data-pos', 'hidden');
        }
      });
      dots.forEach((dot, i) => {
        dot.classList.toggle('active', i === current);
      });
    }

    function goTo(index) {
      current = (index + total) % total;
      updatePositions();
    }

    function startAuto() {
      autoPlay = setInterval(() => goTo(current + 1), 4000);
    }

    function stopAuto() {
      clearInterval(autoPlay);
    }

    prevBtn.addEventListener('click', () => { stopAuto(); goTo(current - 1); startAuto(); });
    nextBtn.addEventListener('click', () => { stopAuto(); goTo(current + 1); startAuto(); });

    // Click side slides to navigate
    slides.forEach(slide => {
      slide.addEventListener('click', () => {
        const pos = slide.getAttribute('data-pos');
        if (pos === 'left') { stopAuto(); goTo(current - 1); startAuto(); }
        else if (pos === 'right') { stopAuto(); goTo(current + 1); startAuto(); }
      });
    });

    carousel.addEventListener('mouseenter', stopAuto);
    carousel.addEventListener('mouseleave', startAuto);

    updatePositions();
    startAuto();
  }

  // ===== Lightbox =====
  const lightbox = document.getElementById('lightbox');
  const lightboxImg = document.getElementById('lightbox-img');

  if (lightbox) {
    document.querySelectorAll('.figure img, .gallery-grid img, .team-member .photo img').forEach(img => {
      img.addEventListener('click', () => {
        lightboxImg.src = img.src;
        lightboxImg.alt = img.alt;
        lightbox.classList.add('active');
      });
    });

    lightbox.addEventListener('click', () => {
      lightbox.classList.remove('active');
    });

    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') {
        lightbox.classList.remove('active');
      }
    });
  }
});
