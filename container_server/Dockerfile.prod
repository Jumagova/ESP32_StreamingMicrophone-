# Use a Node.js base image
FROM node:18.14.0-alpine


# Set the working directory to /app
WORKDIR /app

# Copy package.json and package-lock.json to the working directory
COPY package*.json ./

# Install dependencies
RUN npm install


# Copy the rest of the application code to the working directory
COPY . .


# Expose the ports used by the server
EXPOSE 8080
EXPOSE 8081

# Start the server
CMD ["npm", "start"]
