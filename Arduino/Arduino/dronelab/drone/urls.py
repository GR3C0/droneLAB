from django.conf.urls import include, url
from . import views

urlpatterns = [
        url(r'^$', views.post_list),
        url(r'^volar/$', views.volar),
        url(r'^volar/volar2$', views.raspduino),
]
