import React from 'react';
import { render } from '@testing-library/react';
import App from './Stage';

test('renders learn react link', () => {
  const { getByText } = render(<Stage />);
  const linkElement = getByText(/learn react/i);
  expect(linkElement).toBeInTheDocument();
});
