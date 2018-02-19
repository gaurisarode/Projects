<%@ include file="header.jsp" %>
<%@ include file="sidebar-user.jsp" %>
 <section id="main-content">
   <section class="wrapper">
   <div class="alert alert-info fade in">
                                  <button data-dismiss="alert" class="close close-sm" type="button">
                                      <i class="icon-remove"></i>
                                  </button>
                                  Welcome <strong><%=session.getAttribute("email") %></strong>
                                 <div align="right"><a href="<%=request.getContextPath() %>/Admin?action=user-login">Log Out</a></div>
                              </div>
     
                  </section>
                  </section>
                  <%@ include file="footer.jsp" %>