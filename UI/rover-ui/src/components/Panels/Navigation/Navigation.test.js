import React from 'react';
import { render } from '@testing-library/react';
import Navigation from './Navigation';

test('renders Navigation without a problem', () => {
  const { getByText } = render(<Navigation />);
  const linkElement = getByText(/Hello/i);
  expect(linkElement).toBeInTheDocument();
});
