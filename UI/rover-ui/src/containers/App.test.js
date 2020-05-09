import React from 'react';
import { render, screen } from '@testing-library/react';
import App from './App';

test('renders App without a problem', () => {
  render(<App />);

  screen.getByText((content, node) => {
    const hasText = (node) => node.textContent === "OzU Rover, 2020 UI Debut";
    const nodeHasText = hasText(node);
    const childrenDontHaveText = Array.from(node.children).every(
      (child) => !hasText(child)
    );

    return nodeHasText && childrenDontHaveText;
  });
});
