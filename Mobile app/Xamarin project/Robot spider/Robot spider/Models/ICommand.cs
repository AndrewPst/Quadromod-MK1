using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Input;

namespace Robot_spider.Models
{
    //public interface ICommand
    //{
    //    void Execute(object arg);
    //    bool CanExecute(object arg);
    //    event EventHandler CanExecuteChanged;
    //}


    public class DelegateCommand : ICommand
    {
        public event EventHandler CanExecuteChanged;

        Action _action;

        public DelegateCommand(Action action)
        {
            _action = action;
        }

        public bool CanExecute(object parameter)
        {
            return true;
        }

        public void Execute(object parameter)
        {
            _action();
        }
    }

    public class DelegateCommand<T> : ICommand
    {
        public event EventHandler CanExecuteChanged;

        Action<T> _action;

        public DelegateCommand(Action<T> action)
        {
            _action = action;
        }

        public bool CanExecute(object parameter)
        {
            return true;
        }

        public void Execute(object parameter)
        {
            _action((T)parameter);
        }
    }

}
