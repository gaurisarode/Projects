<!--sidebar start-->
      <aside>
          <div id="sidebar"  class="nav-collapse ">
              <!-- sidebar menu start-->
              <ul class="sidebar-menu">                
                  <li class="active">
                      <a class="" href="<%=request.getContextPath() %>/Admin">
                          <i class="icon_house_alt"></i>
                          <span>Dashboard</span>                      </a>                  </li>
				  <li class="sub-menu">
                      <a href="javascript:;" class="">
                          <i class="icon_document_alt"></i>
                          <span>Categories</span>
                          <span class="menu-arrow arrow_carrot-right"></span>                      </a>
                      <ul class="sub">
                          <li><a href="<%=request.getContextPath() %>/Admin?action=create-category">Create Category</a></li>                          
                          <li><a href="<%=request.getContextPath() %>/Admin?action=manage-category">Manage Category</a></li>
                      </ul>
                  </li>       
                 
                  <li class="sub-menu">
                      <a href="javascript:;" class="">
                          <i class="icon_table"></i>
                          <span>Users</span>
                          <span class="menu-arrow arrow_carrot-right"></span>                      </a>
                      <ul class="sub">
                          <li><a href="<%=request.getContextPath() %>/Admin?action=create-user">Create User</a> </li>
						  <li><a href="<%=request.getContextPath() %>/Admin?action=manage-user">Manage User</a> </li>
						 
                      </ul>
                  </li>
              </ul>
              <!-- sidebar menu end-->
          </div>
      </aside>
      <!--sidebar end-->