import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const ReadingProgressBar = () => {
  const [progress, setProgress] = useState(0);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const handleScroll = () => {
      const scrollTop = window.scrollY;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const scrollPercent = scrollTop / docHeight;
      setProgress(Math.min(100, Math.max(0, scrollPercent * 100)));

      setIsVisible(scrollTop > 100);
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    handleScroll();

    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <div
      className={clsx(
        styles.progressContainer,
        isVisible && styles.visible
      )}
    >
      <div className={styles.progressBar}>
        <div
          className={styles.progressFill}
          style={{ width: `${progress}%` }}
        />
      </div>
      <span className={styles.progressText}>
        {Math.round(progress)}% read
      </span>
    </div>
  );
};

export default ReadingProgressBar;
