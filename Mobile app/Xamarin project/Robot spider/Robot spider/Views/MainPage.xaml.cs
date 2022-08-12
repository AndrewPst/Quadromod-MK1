using Robot_spider.ViewModels;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xamarin.Forms;

namespace Robot_spider.Views
{
    public partial class MainPage : ContentPage
    {
        public MainPage()
        {
            InitializeComponent();
            ((MainPageViewModel)BindingContext)._navigation = Navigation;
        }
        protected override bool OnBackButtonPressed()
        {
            return true;
        }
    }
}
