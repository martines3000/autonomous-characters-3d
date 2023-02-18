###############################
# Stage 1 - the build process #
###############################

FROM rust:1.67 as builder

RUN apt-get update && apt-get install -y clang

WORKDIR /app

# Install trunk
RUN cargo install --locked --version 0.16.0 trunk

COPY . .

# Install wasm-bindgen
RUN cargo install --version 0.2.84 wasm-bindgen-cli

# Add wasm32-unknown-unknown target
RUN rustup target add wasm32-unknown-unknown

# Build the app in release mode
RUN trunk build --release

########################################
# Stage 2 - the production environment #
########################################
FROM nginx:1.23.3-alpine

# Remove default nginx website
RUN rm -rf /usr/share/nginx/html/*

# Copy nginx config file
COPY ./web/nginx.conf /etc/nginx/nginx.conf

# Copy dist folder fro build stage to nginx public folder
COPY --from=builder /app/dist /usr/share/nginx/html

# Start NgInx service
CMD ["nginx", "-g", "daemon off;"]