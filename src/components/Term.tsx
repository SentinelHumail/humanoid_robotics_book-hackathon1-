import React from 'react';
import styles from './Term.module.css';

interface TermProps {
  word: string;
  definition: string;
  children?: React.ReactNode;
}

export default function Term({ word, definition, children }: TermProps) {
  return (
    <span className={styles.term} title={definition}>
      {children || word}
    </span>
  );
}
