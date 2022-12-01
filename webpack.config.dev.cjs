const path = require('path');

module.exports = {
  entry: {
    'mindar-image': './mindar/image-target/index.js',
    'mindar-image-aframe': './mindar/image-target/aframe.js',
    'mindar-image-three': './mindar/image-target/three.js',
    'mindar-face': './mindar/face-target/index.js',
    'mindar-face-aframe': './mindar/face-target/aframe.js',
    'mindar-face-three': './mindar/face-target/three.js'
  },
  mode: 'development',
  devtool: 'inline-source-map',
  output: {
    filename: '[name].js',
    path: path.resolve(__dirname, 'static', 'mindar'),
    publicPath: ''
  },
  module: {
    rules: [
      {
        test: /\.worker\.js$/,
        use: {
          loader: 'worker-loader',
          options: {
            inline: true,
            name: '[name].js'
          },
        },
      },
      {
	test: /\.s[ac]ss$/i,
        use: [
          'style-loader',
          'css-loader',
	  'sass-loader'
        ]
      },
      {
	test: /\.html$/i,
        use: 'html-loader',
      },
    ],
  },
  resolve: {
    fallback: {
      fs: false,
      path: false,
      crypto: false
    }
  }
};
