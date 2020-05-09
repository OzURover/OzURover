import React from 'react';
import { render } from '@testing-library/react';
import Stage from './Stage';

test('renders Stage without a problem', () => {
  const { getAllByRole } = render(<Stage />);
  const linkElement = getAllByRole("tab");
  expect(linkElement).toHaveLength(3);
});
