
module.exports = {
  // ... other configuration options
  resolve: {
    fallback: {
      stream: require.resolve('stream-browserify'),
    },
  },
};