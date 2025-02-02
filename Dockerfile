FROM node:lts AS app_builder

WORKDIR /app
COPY . .
RUN yarn && yarn build

FROM nginx

COPY ./nginx/default.conf /etc/nginx/conf.d/default.conf
COPY --from=app_builder /app/build /usr/share/nginx/html

EXPOSE 80
